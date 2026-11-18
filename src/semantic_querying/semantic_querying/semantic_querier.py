import json
import math
from pathlib import Path
import queue
import time
from concurrent.futures import ThreadPoolExecutor

import rclpy
import rclpy.time
import requests
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

OLLAMA_URL = 'http://localhost:11434/api/chat'
MODEL = 'qwen2.5:3b-instruct'


SYSTEM = '''
        You are a guide robot. You get a user request and a JSON list of 
        known objects in the environment, each with an id and a description. 
        
        Infer what the user means and Choose the object that best satisfies the user's request.

        Reply with JSON only: 
        
        {
            "observation_id": "<id from the list>",
            "response": "<your response here>"
        }
        
        or if nothing fits:
        
        {
            "observation_id": null,
            "response": "<your response here>"
        } 

        The response must describe the object associated with the selected ID.
        Copy the ID exactly from that object's line. Do not invent an ID.
        
        If there are multiple possible matches, choose the most relevant one.        
        '''


class SemanticQuerier(Node):

    def __init__(self):
        super().__init__('semantic_querier')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self._request_sub = self.create_subscription(String, '/user_request', self.request_callback, 10)

        self._response_pub = self.create_publisher(String, '/robot_response', 10)
        self._status_pub = self.create_publisher(String, '/robot_status', 10)
                
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._request_queue = queue.Queue()
        self._selector = ThreadPoolExecutor(max_workers=1)

        self._selection_future = None
        self._request_active = False
        self._pending_goal = None
        self._server_deadline = None

        self.create_timer(0.1, self.process_user_request)
        

    def request_callback(self, msg):
        self._request_queue.put(msg.data)

    def process_user_request(self):
        if self._selection_future is not None:
            if not self._selection_future.done():
                return

            future = self._selection_future
            self._selection_future = None

            try:
                observation_id, reply = future.result()
            except Exception as exc:
                self.fail_request(f'Object selection failed: {exc}')
                return

            if observation_id is None:
                self.send_response(reply)
                self.finish_request()
                return

            try:
                record = self.load_observation(observation_id)
                goal = self.make_approach_pose(record['position'])
            except (OSError, ValueError, KeyError, TypeError, IndexError,
                    tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as exc:
                self.fail_request(f'Could not prepare the navigation goal: {exc}')
                return

            self.send_response(reply)
            self._pending_goal = goal
            self._server_deadline = time.monotonic() + 10.0
            return

        if self._pending_goal is not None:
            if self._action_client.server_is_ready():
                goal = self._pending_goal
                self._pending_goal = None
                goal.header.stamp = self.get_clock().now().to_msg()
                try:
                    self.send_goal([goal])
                except Exception as exc:
                    self.fail_request(f'Could not send the navigation goal: {exc}')
            elif time.monotonic() >= self._server_deadline:
                self._pending_goal = None
                self.fail_request('Navigation is unavailable. Start Nav2 and try again.')
            return

        if self._request_active:
            return

        try:
            request = self._request_queue.get_nowait()
        except queue.Empty:
            return

        self._request_active = True
        self._selection_future = self._selector.submit(self.select_object, request)

    def select_object(self, request):

        path = Path.home() / '.ros' / 'semantic_observations.jsonl'
        
        observations = []
        with path.open(encoding='utf-8') as file:
            for line in file:
                record = json.loads(line)
                observations.append(record)  

        if not observations:
            raise ValueError('No observations found')

        listing = '\n'.join(f'{o["observation_id"]}: {o["description"]}' for o in observations)
        
        response = requests.post(OLLAMA_URL, json={
            'model': MODEL,
            'stream': False,
            'format': 'json',            
            'options': {'temperature': 0},
            'messages': [
                {'role': 'system', 'content': SYSTEM},
                {'role': 'user', 'content': f'Objects:\n{listing}\n\nRequest: {request}'},
            ],
        }, timeout=60)

        response.raise_for_status()
        result = json.loads(response.json()['message']['content'])
        chosen = result['observation_id']
        reply = result['response']
        if not isinstance(reply, str) or not reply.strip():
            raise ValueError('The language model returned an empty response')
        valid = {o['observation_id'] for o in observations}
        if chosen is not None and (not isinstance(chosen, str) or chosen not in valid):
            raise ValueError('The language model selected an unknown observation')
        return chosen, reply.strip()


    def load_observation(self, observation_id):
        path = Path.home() / '.ros' / 'semantic_observations.jsonl'

        matches = []
        with path.open(encoding='utf-8') as file:
            for line in file:
                record = json.loads(line)
                if record.get('observation_id') == observation_id:
                    matches.append(record)

        if len(matches) != 1:
            raise ValueError(
                f'Expected one observation with ID {observation_id}, found {len(matches)}'
            )

        record = matches[0]
        if record.get('map_id') != 'my_world' or record.get('frame') != 'map':
            raise ValueError('Observation is not in the active map frame')

        return record

    def make_approach_pose(self, object_position):
        object_x, object_y = object_position[:2]

        tf_robot_map = self._tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())

        
        robot_x = tf_robot_map.transform.translation.x
        robot_y = tf_robot_map.transform.translation.y

        dx = robot_x - object_x
        dy = robot_y - object_y
        distance = math.hypot(dx, dy)

        if distance < 1e-3:
            raise ValueError('Robot is too close to the object to choose an approach side')

        # distance away from object
        standoff = 1.0
        goal_x = object_x + standoff * dx / distance
        goal_y = object_y + standoff * dy / distance

        # Face the object from the approach pose.
        yaw = math.atan2(object_y - goal_y, object_x - goal_x)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal


    def send_goal(self, poses: list[PoseStamped]):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} m | ', throttle_duration_sec=1.0)

    def goal_response_callback(self, future):
        try:
            self._goal_handle = future.result()
        except Exception as exc:
            self.fail_request(f'Could not send the navigation goal: {exc}')
            return
        if not self._goal_handle.accepted:
            self.fail_request('Nav2 rejected the navigation goal.')
            return

        self.get_logger().info('Goal Accepted')
        try:
            self._get_result_future = self._goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        except Exception as exc:
            self.fail_request(f'Could not request the navigation result: {exc}')

    def get_result_callback(self, future):
        try:
            status = future.result().status
        except Exception as exc:
            self.fail_request(f'Could not get the navigation result: {exc}')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached destination!')
            self.send_response('Reached destination!')
        else:
            self.get_logger().warn(f'Navigation ended with status code {status}')
            self.send_response(f'Navigation ended with status code {status}')

        self.finish_request()

    def send_response(self, request):
        msg = String()
        msg.data = request
        self._response_pub.publish(msg)

    def finish_request(self):
        self._request_active = False
        msg = String()
        msg.data = 'finished'
        self._status_pub.publish(msg)

    def fail_request(self, message):
        self.get_logger().error(message)
        self.send_response(message)
        self.finish_request()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SemanticQuerier()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node._selector.shutdown(wait=True, cancel_futures=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
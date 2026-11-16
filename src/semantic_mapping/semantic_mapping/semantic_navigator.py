import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_ros
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
import sys
import json
from pathlib import Path
import math
from geometry_msgs.msg import PoseStamped
import json
import requests
from std_msgs.msg import String
import queue
import threading
from concurrent.futures import ThreadPoolExecutor

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


class SemanticNavigator(Node):

    def __init__(self):
        super().__init__('semantic_navigator')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        # self.create_subscription(String, '/user_request', self.request_callback, 10)

                
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._request_queue = queue.Queue()
        self._request_finished = threading.Event()
        self._request_finished.set()

        self._selector = ThreadPoolExecutor(max_workers=1)
        self._selection_future = None

        self._input_thread = threading.Thread(target=self.read_user_request, daemon=True)
        self._input_thread.start()

        self.create_timer(0.1, self.process_user_request)



    # def request_callback(self, msg):  
    #     observation_id = self.select_object(msg.data)
    #     if observation_id is None:
    #         self.get_logger().warn("No valid observation selected")
    #         return
    #     record = self.load_observation(observation_id)
    #     self.get_logger().info(f'Loaded observation: {record}')
    #     goal = self.make_approach_pose(record['position'])
    #     if goal is None:
    #         self.get_logger().warn('TF not ready')
    #         return
    #     self.send_goal([goal])


    def read_user_request(self):
        while True:
            self._request_finished.wait()
            request = input("What are you looking for? (or 'exit'): ")
            if request.lower() == 'exit':
                self._request_queue.put(None)
                break
            if not request:
                continue
            self._request_finished.clear()
            self._request_queue.put(request)

    def process_user_request(self):
        if self._selection_future is not None:
            if not self._selection_future.done():
                return

            future = self._selection_future
            self._selection_future = None

            observation_id = future.result()

            if observation_id is None:
                self.get_logger().warn("No matching observation found")
                self._request_finished.set()
                return

            record = self.load_observation(observation_id)
            self.get_logger().info(f'Loaded observation: {record}')

            goal = self.make_approach_pose(record['position'])

            if goal is None:
                self._request_finished.set()
                return
            
            self.send_goal([goal])
            return

        try:
            request = self._request_queue.get_nowait()
        except queue.Empty:
            return

        if request is None:
            self.get_logger().info('Exiting.')
            rclpy.shutdown()
            return

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
        chosen = json.loads(response.json()['message']['content']).get('observation_id')
        self.get_logger().info(f'{json.loads(response.json()["message"]["content"]).get('response')}')
        valid = {o['observation_id'] for o in observations}
        return chosen if chosen in valid else None  


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

        try:
            tf_robot_map = self._tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            self.get_logger().warning(f'Cannot get robot transform')
            return

        
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

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f} m | ', throttle_duration_sec=1.0)
            # f'Number of recovery executions: {feedback.number_of_recoveries}'

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn('Goal Rejected')
            self._request_finished.set() 
            return

        self.get_logger().info('Goal Accepted')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('Reached destination!')
        else:
            self.get_logger().warn(f'Navigation ended with status code {status}')

        self._request_finished.set() 



def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SemanticNavigator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
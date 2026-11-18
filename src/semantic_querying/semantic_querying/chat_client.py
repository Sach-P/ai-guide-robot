import queue
import threading
import rclpy
import streamlit as st
from rclpy.node import Node
from std_msgs.msg import String

class ChatClient(Node):
    def __init__(self):
        super().__init__('chat_client')
        self.create_subscription(String, '/robot_response', self.response_callback, 10)
        self.create_subscription(String, '/robot_status', self.status_callback, 10)
        self._publisher = self.create_publisher(String, '/user_request', 10)
        self._events = queue.Queue()

    def response_callback(self, msg):
        self._events.put(('response', msg.data))

    def status_callback(self, msg):
        self._events.put(('status', msg.data))

    def send_request(self, request):
        msg = String()
        msg.data = request
        self._publisher.publish(msg)


@st.cache_resource
def create_client():
    rclpy.init()
    node = ChatClient()

    thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    thread.start()

    return node


st.title('AI Guide Robot 🤖')

if 'messages' not in st.session_state:
    st.session_state.messages = []

if 'pending_request' not in st.session_state:
    st.session_state.pending_request = False

client = create_client()

@st.fragment(run_every='1s')
def show_chat():
    while True:
        try:
            kind, value = client._events.get_nowait()
        except queue.Empty:
            break
        if kind == 'response':
            st.session_state.messages.append({'role': 'assistant', 'content': value})
        elif kind == 'status' and value == 'finished':
            st.session_state.pending_request = False

    for message in st.session_state.messages:
        with st.chat_message(message['role']):
            st.write(message['content'])

    if st.session_state.pending_request:
        st.caption('Waiting for the robot to finish...')

    message = st.chat_input(
        'Where would you like to go?',
        disabled=st.session_state.pending_request,
    )
    if message:
        if client._publisher.get_subscription_count() == 0:
            st.error('The semantic querier is not connected.')
        else:
            st.session_state.messages.append({'role': 'user', 'content': message})
            st.session_state.pending_request = True
            client.send_request(message)
            st.rerun()


show_chat()
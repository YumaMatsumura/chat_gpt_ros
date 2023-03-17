import os

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from chat_gpt_ros.chat_gpt import ChatGPT
from chat_gpt_msgs.srv import AskChatGpt

class ChatGPTROS(Node):
    def __init__(self):
        super().__init__('chat_gpt_node')
        self.declare_parameter('max_tokens', 1024)
        self.declare_parameter('n', 1)
        self.declare_parameter('temperature', 0.5)
        self.srv = self.create_service(AskChatGpt, 'ask_chat_gpt', self.ask_chat_gpt_callback)
        
    def ask_chat_gpt_callback(self, request, response):
        max_tokens = self.get_parameter('max_tokens').value
        n = self.get_parameter('n').value
        temperature = self.get_parameter('temperature').value
        gpt = ChatGPT(api_key=os.environ["OPENAI_API_KEY"], max_tokens=max_tokens, n=n, temperature=temperature)
        response.response_message = gpt.ask_gpt(request.request_message)
        
        return response
        
def main(args=None):
    rclpy.init(args=args)
    try:
        chat_gpt_ros = ChatGPTROS()
        executor = SingleThreadedExecutor()
        executor.add_node(chat_gpt_ros)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            chat_gpt_ros.destroy_node()
    finally:
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()

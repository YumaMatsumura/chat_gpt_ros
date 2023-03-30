import os

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from chat_gpt_ros.chat_gpt import ChatGPT
from chat_gpt_msgs.srv import AskChatGpt

class ChatGPTROS(Node):
    def __init__(self):
        super().__init__('chat_gpt_node')
        self.declare_parameter('system_message', '')
        self.declare_parameter('model', 'gpt-3.5-turbo')
        self.declare_parameter('max_tokens', 1024)
        self.declare_parameter('max_total_tokens', 3000)
        self.declare_parameter('number_of_response', 1)
        self.declare_parameter('timeout', 60)
        self.declare_parameter('temperature', 0.5)
        self.declare_parameter('use_system_role', False)
        self.declare_parameter('keep_conversation_history', True)
        self.srv = self.create_service(AskChatGpt, 'ask_chat_gpt', self.ask_chat_gpt_callback)
        
        self.generate_chat_gpt_instance()
        
    def generate_chat_gpt_instance(self):
        # Get params
        system_message = self.get_parameter('system_message').value
        model = self.get_parameter('model').value
        max_tokens = self.get_parameter('max_tokens').value
        max_total_tokens = self.get_parameter('max_total_tokens').value
        number_of_response = self.get_parameter('number_of_response').value
        timeout = self.get_parameter('timeout').value
        temperature = self.get_parameter('temperature').value
        use_system_role = self.get_parameter('use_system_role').value
        keep_conversation_history = self.get_parameter('keep_conversation_history').value
        
        # Generate instance
        self.gpt = ChatGPT(api_key=os.environ['OPENAI_API_KEY'], 
                      system_message=system_message,
                      model=model,
                      max_total_tokens=max_total_tokens,
                      max_tokens=max_tokens, 
                      number_of_response=number_of_response, 
                      timeout=timeout,
                      temperature=temperature,
                      use_system_role=use_system_role,
                      keep_conversation_history=keep_conversation_history)
                      
                      
    def ask_chat_gpt_callback(self, request, response):
        try:
            # Request to chatGPT
            response.response_message = self.gpt.ask_gpt(request.request_message)
            response.result = True
            self.get_logger().info("ChatGPT Output: " + response.response_message)
            
            return response
            
        except Exception as e:
            self.get_logger().error('chat_gpt_ros has failed %r' & (e,))
            response.result = False
            
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

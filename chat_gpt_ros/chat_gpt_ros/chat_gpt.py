import openai

class ChatGPT:
    def __init__(self, api_key, system_message, model='gpt-3.5-turbo', max_total_tokens=3000, 
                 max_tokens=1024, number_of_response=1, timeout=60, stop=None, 
                 temperature=0.5, use_system_role=False, keep_conversation_history=True):
        self.api_key = api_key
        self.sys_msg = system_message
        self.model = model
        self.max_total_tokens = max_total_tokens
        self.max_tokens = max_tokens
        self.number_of_response = number_of_response
        self.timeout = timeout
        self.stop = stop
        self.temperature = temperature
        self.use_system_role = use_system_role
        self.keep_conversation_history = keep_conversation_history
        self.messages = []
        
    def ask_gpt(self, message):
        # APIキーの設定
        openai.api_key = self.api_key
        
        # system roleの設定
        if self.use_system_role:
            system_message = {'role': 'system', 'content': self.sys_msg}
            self.messages.append(system_message)
        
        # リクエストメッセージの定義
        user_message = {'role': 'user', 'content': message}
        self.messages.append(user_message)
        
        try:
            # 応答設定
            completion = openai.ChatCompletion.create(
                model       = self.model,
                messages    = self.messages,
                max_tokens  = self.max_tokens,
                n           = self.number_of_response,
                timeout     = self.timeout,
                stop        = self.stop,
                temperature = self.temperature,
            )
        
            # 応答
            response = completion.choices[0].message.content
        
            # 会話履歴の保持
            if self.keep_conversation_history:
                gpt_message = {'role': 'assistant', 'content': response}
                self.messages.append(gpt_message)
            
                if completion['usage']['total_tokens'] > self.max_total_tokens:
                    conversation_start_index = 0
                    if self.use_system_role:
                        conversation_start_index = 1
                    self.messages.pop(conversation_start_index)
                    self.messages.pop(conversation_start_index)
            else:
                self.messages.clear()
                
            return response
                
        except Exception as e:
            raise

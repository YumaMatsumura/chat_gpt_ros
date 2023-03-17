import openai

class ChatGPT:
    def __init__(self, api_key, max_tokens=1024, n=1, stop=None, temperature=0.5):
        self.api_key = api_key
        self.max_tokens = max_tokens
        self.n = n
        self.stop = stop
        self.temperature = temperature
        
    def ask_gpt(self, message):
        # APIキーの設定
        openai.api_key = self.api_key
        
        # 応答設定
        completion = openai.ChatCompletion.create(
            model    = "gpt-3.5-turbo",     # モデルを選択
            messages = [
                {"role": "user", "content": message},
            ],
    
            max_tokens  = self.max_tokens,
            n           = self.n,
            stop        = self.stop,
            temperature = self.temperature,
        )
        
        # 応答
        response = completion.choices[0].message.content
        
        return response

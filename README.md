# chat_gpt_ros
## 動作確認環境
- Ubuntu22.04
- ROS2 Humble

## 環境セットアップ
1. openaiモジュールをインストールする
   ```bash
   pip3 install openai
   ```
   
2. ros2ワークスペースに本パッケージをクローンする
   ```bash
   mkdir -p ~/gpt_ws/src && cd ~/gpt_ws/src
   ```
   ```bash
   git clone https://github.com/YumaMatsumura/chat_gpt_ros.git
   ```
   
3. ビルドする
   ```bash
   cd ~/gpt_ws
   ```
   ```bash
   colcon build
   ```
   
4. openaiのapi_keyを環境変数に設定する
   ```bash
   export OPENAI_API_KEY="XXX"
   ```
   
## 動作確認
1. chat_gpt_ros2を起動する
   ```bash
   ros2 run chat_gpt_ros chat_gpt_ros
   ```
   
2. chatGPTに聞きたい内容をサービスとしてコールする
   ```bash
   ros2 service call /ask_chat_gpt chat_gpt_msgs/srv/AskChatGpt "{request_message: \"ROSとは何か説明してください。\"}"
   ```

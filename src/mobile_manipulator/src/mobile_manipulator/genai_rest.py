import os
import requests
import json

class MockFunctionCall:
    def __init__(self, name, args):
        self.name = name
        self.args = args

class MockPart:
    def __init__(self, function_call=None):
        self.function_call = function_call
    
    @staticmethod
    def from_function_response(name, response):
        return {"functionResponse": {"name": name, "response": response}}

class MockResponse:
    def __init__(self, text="", parts=None):
        self.text = text
        self.parts = parts or []

class ChatSession:
    def __init__(self, api_key, model_name, tools, sys_prompt):
        # Convert legacy type_ to type, and handle REST formatting
        tools_str = json.dumps(tools).replace('"type_"', '"type"')
        self.tools = json.loads(tools_str)
        # Dynamically verify / discover a supported model!
        clean_model = model_name.replace("models/", "")
        try:
            models_resp = requests.get(
                f"https://generativelanguage.googleapis.com/v1beta/models?key={api_key}",
                timeout=10
            ).json()
            available = [m["name"].replace("models/", "") for m in models_resp.get("models", [])
                        if "generateContent" in m.get("supportedGenerationMethods", [])]

            if available and clean_model not in available:
                fallbacks = [m for m in available if "flash" in m.lower()] + \
                            [m for m in available if "pro" in m.lower()] + available
                clean_model = fallbacks[0]
                print(f"[GenAI REST] '{model_name}' not found; using '{clean_model}'")
        except Exception as e:
            print(f"[GenAI REST] Model discovery failed ({e}); proceeding with '{clean_model}'")

        self.url = f"https://generativelanguage.googleapis.com/v1beta/models/{clean_model}:generateContent?key={api_key}"
        self.history = []
        self.sys_prompt = sys_prompt
        
    def send_message(self, content):
        if isinstance(content, str):
            self.history.append({"role": "user", "parts": [{"text": content}]})
        else:
            self.history.append({"role": "user", "parts": [content]})
            
        payload = {
            "contents": self.history,
            "tools": self.tools,
            "system_instruction": {"parts": [{"text": self.sys_prompt}]}
        }
        res = requests.post(self.url, json=payload).json()
        
        if "error" in res:
            raise Exception(f"Gemini API Error: {res['error']['message']}")
            
        msg = res["candidates"][0]["content"]
        # Append the raw response back to history exactly as standard GenAI
        self.history.append(msg)
        
        text = ""
        mock_parts = []
        for p in msg.get("parts", []):
            if "text" in p:
                text += p["text"]
            if "functionCall" in p:
                fc = p["functionCall"]
                mock_parts.append(MockPart(MockFunctionCall(fc["name"], fc.get("args", {}))))
                
        return MockResponse(text, mock_parts)

class GenerativeModel:
    def __init__(self, model_name, tools, system_instruction):
        self.api_key = os.environ.get("GEMINI_API_KEY", "")
        self.model_name = model_name
        self.tools = tools
        self.sys_prompt = system_instruction
        
    def start_chat(self):
        return ChatSession(self.api_key, self.model_name, self.tools, self.sys_prompt)

class types:
    Part = MockPart

def configure(**_):
    pass

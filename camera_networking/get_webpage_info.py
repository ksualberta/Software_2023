import requests

response = requests.get("http://127.0.0.1:5500/Software_2023/website_interface_example/index.html")
print(response.content.decode(encoding='utf-8'))
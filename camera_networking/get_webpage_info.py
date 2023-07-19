import requests

response = requests.get("http://127.0.0.1:5500/Software_2023/website_interface_example/index.html")
response_body = response.content.decode("utf-8").split("\n")
print(response_body)
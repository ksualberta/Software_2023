import requests
import re

def getLocation(response_body:list)->list:
    searchParam = r'[-]*[0-9]*[.]+[0-9]{4}[a-zA-Z]{1}'
    coordinates = re.findall(pattern=searchParam,string=response_body)
    return coordinates

response = requests.get("http://127.0.0.1:5500/Software_2023/website_interface_example/index.html")
response_body = response.content.decode("utf-8")
print(getLocation(response_body))
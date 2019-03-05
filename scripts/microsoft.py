

import urllib2
# If you are using Python 3+, import urllib instead of urllib2

import json 


data =  {

        "Inputs": {

                "input1":
                {
                    "ColumnNames": ["symboling", "normalized-losses", "make", "fuel-type", "aspiration", "num-of-doors", "body-style", "drive-wheels", "engine-location", "wheel-base", "length", "width", "height", "curb-weight", "engine-type", "num-of-cylinders", "engine-size", "fuel-system", "bore", "stroke", "compression-ratio", "horsepower", "peak-rpm", "city-mpg", "highway-mpg", "price"],
                    "Values": [ [ "0", "0", "value", "value", "value", "value", "value", "value", "value", "0", "0", "0", "0", "0", "value", "value", "0", "value", "0", "0", "0", "0", "0", "0", "0", "0" ], [ "0", "0", "value", "value", "value", "value", "value", "value", "value", "0", "0", "0", "0", "0", "value", "value", "0", "value", "0", "0", "0", "0", "0", "0", "0", "0" ], ]
                },        },
            "GlobalParameters": {
}
    }

body = str.encode(json.dumps(data))

url = 'https://ussouthcentral.services.azureml.net/workspaces/756a7a08de6e417f946789e47bb73d54/services/b61923b5e3054ca39812992fdb74b224/execute?api-version=2.0&details=true'
api_key = 'p2jNaR4sOrfIPWOHbhiZvWOeL0XowmkRbAF3vYLH/JsDVj0oJj9xIPxz1OF2FG0mHZEiVDIIp/jDWCz9n5J4TQ==' # Replace this with the API key for the web service
headers = {'Content-Type':'application/json', 'Authorization':('Bearer '+ api_key)}

req = urllib2.Request(url, body, headers) 

try:
    response = urllib2.urlopen(req)

    # If you are using Python 3+, replace urllib2 with urllib.request in the above code:
    # req = urllib.request.Request(url, body, headers) 
    # response = urllib.request.urlopen(req)

    result = response.read()
    print(result) 
except urllib2.HTTPError, error:
    print("The request failed with status code: " + str(error.code))

    # Print the headers - they include the requert ID and the timestamp, which are useful for debugging the failure
    print(error.info())

    print(json.loads(error.read()))                 


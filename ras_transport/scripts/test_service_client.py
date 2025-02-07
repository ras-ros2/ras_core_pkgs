from ras_transport.interfaces.TransportWrapper import TransportServiceClient
import json
import time


def main():
    transport_service_client = TransportServiceClient("test_service")
    transport_service_client.connect_with_retries()
    payload = {
        "msg": "hello"
    }
    req = json.dumps(payload)
    transport_service_client.loop()
    res = transport_service_client.call(req)
    print(f"response is {res}")
    transport_service_client.__del__()
    
    
if __name__ == "__main__":
    main()

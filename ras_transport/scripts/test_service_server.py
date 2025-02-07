from ras_transport.interfaces.TransportWrapper import TransportServiceServer
import json
import time

def response_cb(req):
    print(f"Received req: {req}")
    payload = {
        "success": True
    }
    return json.dumps(payload)

def main():
    transport_service_server = TransportServiceServer("test_service", response_cb)
    transport_service_server.connect_with_retries()
    while True:
        transport_service_server.loop()
        time.sleep(0.1) # for multi threading
    
if __name__ == "__main__":
    main()

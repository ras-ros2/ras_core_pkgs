import socket

def check_if_bindable(ip, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind((ip, port))
        s.close()
        return True
    except:
        s.close()
        return False
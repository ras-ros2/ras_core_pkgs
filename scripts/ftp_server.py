#!/usr/bin/env python3

from ras_common.transport.TransportServer import TransportFTPServer
import rclpy
def main(args=None):
    rclpy.init(args=args)
    ftp_server = TransportFTPServer("real")
    try:
        while rclpy.ok():
            rclpy.spin_once(ftp_server.ftpserver)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect
        ftp_server.ftpserver.safe_kill()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import concurrent.futures
import socket

import paramiko

SUBNET = "10.155.234."
MY_IP = socket.gethostbyname(socket.gethostname())

SSH_USERNAME = "ubuntu"
SSH_PASSWORD = "robot1234"


def check_ssh_port(ip):
    # Exclude your own laptop
    if ip == MY_IP:
        return None

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.5)  # Fast timeout so it does not hang

    try:
        # Check if port 22 is accepting connections
        if sock.connect_ex((ip, 22)) == 0:
            return ip
    except Exception:
        pass
    finally:
        sock.close()

    return None


def get_ros_domain_id(ip):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(ip, username=SSH_USERNAME, password=SSH_PASSWORD, timeout=3)
        _, stdout, _ = client.exec_command(
            "grep -oP 'ROS_DOMAIN_ID=\\\"?\\K[0-9]+' "
            "/etc/turtlebot4/setup.bash 2>/dev/null"
        )
        return stdout.read().decode().strip()
    except Exception as e:
        print(f"  SSH failed for {ip}: {e}")
        return None
    finally:
        client.close()


def main():
    print(f"Scanning subnet {SUBNET}x for open SSH ports...")
    ips = [f"{SUBNET}{i}" for i in range(1, 255)]

    # Run multiple checks at the same time to speed it up
    with concurrent.futures.ThreadPoolExecutor(max_workers=50) as executor:
        results = list(executor.map(check_ssh_port, ips))

    ssh_hosts = [ip for ip in results if ip]
    print(f"Found {len(ssh_hosts)} hosts with SSH open. Checking for TurtleBots...")

    for ip in ssh_hosts:
        domain_id = get_ros_domain_id(ip)
        if domain_id:
            print(f"TurtleBot: {ip}  ROS_DOMAIN_ID={domain_id}")


if __name__ == "__main__":
    main()

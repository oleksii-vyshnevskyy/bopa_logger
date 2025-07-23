import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.logging import get_logger
import subprocess
import datetime
from datetime import datetime
import tarfile
import os
from pathlib import Path
import paramiko

class LogCollector(Node):
    def __init__(self):
        super().__init__('log_collector')
        # action-handler
        self.command_dispatch = {
            'all': self.handler_all,
            'latest': self.handler_latest,
            'date': self.handler_date,
            # Add more command handlers here
        }

        # read configuration from config.yaml
        self.declare_parameter('remote_host', '')
        self.declare_parameter('remote_user', '')
        self.declare_parameter('remote_pass', '')
        self.declare_parameter('remote_path', '')
        self.declare_parameter('transfer_method', '')

        self.remote_host = self.get_parameter('remote_host').value
        self.remote_user = self.get_parameter('remote_user').value
        self.remote_pass = self.get_parameter('remote_pass').value
        self.remote_path = self.get_parameter('remote_path').value
        self.transfer_method = self.get_parameter('transfer_method').value

        # subscribe to topik and wait for command
        self.subscription = self.create_subscription(String, '/log_trigger', self.handle_trigger, 10)

    def handle_trigger(self, msg):
        """
        Trigger that fired when data is received.
        :param msg:
        :return:
        """
        cmd_parts = msg.data.split()
        if not cmd_parts:
            self.get_logger().warn("Empty command received.")
            return
        cmd = cmd_parts[0]
        args = cmd_parts[1:] if len(cmd_parts) > 1 else []

        callback = self.command_dispatch.get(cmd)
        if callback:
            callback(args)
        else:
            self.get_logger().warn(f"No handler for command: {cmd}")


    def handler_all(self, _):
        """
        Archive every run‐directory *and* every loose .log file under ROS_LOG_DIR.
        Input message Ex : ros2 topic pub /log_trigger std_msgs/msg/String "{data: 'all'}" --once
        """
        # all subfolders
        dirs = self.get_log_dirs()
        # all top‑level .log files
        files = self.get_flat_logs()

        paths = dirs + files
        if not paths:
            self.get_logger().warn("handler_all: no logs or dirs found")
            return

        self.get_logger().info(
            f"handler_all: archiving {len(dirs)} dirs + {len(files)} files"
        )
        self.archive_logs([str(p) for p in paths], "all_logs.tar.gz")

    def handler_latest(self, _):
        """
        Archive the latest run‐directory *and* any loose .log files in it.
        Input message Ex : ros2 topic pub /log_trigger std_msgs/msg/String "{data: 'latest'}" --once
        """
        latest_dir = self.get_latest_dir()
        # also grab any .log files that live beside it
        loose = [
            p for p in self.get_flat_logs()
            if Path(p).stat().st_mtime >= latest_dir.stat().st_mtime
        ]

        paths = [latest_dir] + loose
        self.get_logger().info(
            f"handler_latest: archiving 1 dir + {len(loose)} files"
        )
        self.archive_logs([str(p) for p in paths], "latest_logs.tar.gz")

    def handler_date(self, args):
        """
        Archive every run‐folder and every .log file created on a given date.
        Input message Ex : ros2 topic pub /log_trigger std_msgs/msg/String "{data: 'date 2025-07-20'}" --once
        """
        if not args:
            self.get_logger().error("No date provided for 'date' command.")
            return
        date_prefix = args[0]  # e.g. "2025-07-22"

        # 1) collect matching folders
        dirs = [
            d for d in self.get_log_dirs()
            if d.name.startswith(date_prefix)
        ]

        # 2) collect matching .log files by modification date
        files = []
        for p in self.get_flat_logs():
            # get the file's mtime date as "YYYY-MM-DD"
            file_date = datetime.fromtimestamp(p.stat().st_mtime).date().isoformat()
            if file_date == date_prefix:
                files.append(str(p))

        # combine
        paths = [str(d) for d in dirs] + files
        if not paths:
            self.get_logger().info(f"No logs or folders match date: {date_prefix}")
            return

        # log what we're archiving
        self.get_logger().info(
            f"handler_date: archiving {len(dirs)} folder(s) and {len(files)} file(s) for date {date_prefix}"
        )

        # finally create the archive
        archive_name = f"logs_{date_prefix}.tar.gz"
        self.archive_logs(paths, archive_name)

    def get_ros_log_dir(self):
        """
        If ROS_LOG_DIR is exported, it uses that.
        If not, it falls back to ~/.ros/log automatically for the current user.
        :return: PosixPath(string)
        """
        return Path(os.environ.get("ROS_LOG_DIR", Path.home() / ".ros" / "log"))

    def get_log_dirs(self):
        """
        Return dirs inside ROS_LOG_DIR folder without "latest" directory.
        :return: list[PosixPath(string) ..]
        """
        log_dir = self.get_ros_log_dir()
        return sorted([p for p in log_dir.iterdir() if p.is_dir() and not p.name == "latest"], reverse=True)

    def get_flat_logs(self):
        """
        Return *.log files inside ROS_LOG_DIR folder.
        :return: list[PosixPath(string) ..]
        """
        return list(self.get_ros_log_dir().glob("*.log"))

    def get_latest_dir(self):
        """
        Return latest directory inside ROS_LOG_DIR folder.
        :return: PosixPath(string)
        """
        log_dir = self.get_ros_log_dir()
        link = log_dir / "latest"
        return link.resolve() if link.exists() else self.get_log_dirs()[0]

    def archive_logs(self, paths, archive_name):
        """
        Pack logs to archive.
        :param paths:
        :param archive_name:
        :return:
        """
        log_dir = self.get_ros_log_dir()
        archive_path = log_dir / archive_name
        archive_path.parent.mkdir(parents=True, exist_ok=True)

        with tarfile.open(archive_path, "w:gz") as tar:
            for entry in paths:
                p = Path(entry)
                if not p.exists():
                    self.get_logger().warn(f"archive_logs: skipping missing file {p}")
                    continue
                # add the file, using only its basename inside the archive
                tar.add(str(p), arcname=p.name)
        self.get_logger().info(f"Archived logs to {archive_path}")
        self.send_to_remote(archive_path)
        self.remove_archived_logs(archive_path)

    def remove_archived_logs(self, archive_name: str) -> bool:
        """
        Remove the given archived log file from local disk.
        :param archive_name: filename (e.g. "all_logs.tar.gz") or absolute path
        :returns: True if removal succeeded or file didn’t exist, False on error
        """
        # Resolve to an absolute Path
        candidate = Path(archive_name)
        if not candidate.is_absolute():
            # assume it lives in the configured log directory
            candidate = Path(self.log_directory) / archive_name

        try:
            if candidate.exists():
                candidate.unlink()
                self.get_logger().info(f"Removed archived log: {candidate}")
            else:
                self.get_logger().warn(f"No archived log to remove at: {candidate}")
            return True
        except Exception as e:
            self.get_logger().error(f"Error removing archived log {candidate}: {e}")
            return False


    def send_to_remote(self, archive_path):
        """
        Send logs using preconfigured method
        Configuration used : check config.yaml
        :param archive_path:
        :return:
        """
        if self.transfer_method == "paramiko":
            self.send_paramiko(archive_path)
        else: self.get_logger().warn("Unknown transfer method '{}'".format(self.transfer_method))

    def send_paramiko(self,archive_path):
        """
        Send logs using ssh paramiko.
        Configuration used : check config.yaml
        :param archive_path:
        :return:
        """
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(hostname=self.remote_host, username=self.remote_user, password=self.remote_pass, port=22)
            sftp = ssh.open_sftp()
            filename = os.path.basename(archive_path)
            remote_path = f"{self.remote_path.rstrip('/')}/{filename}"
            sftp.put(str(archive_path), remote_path)
            sftp.close()
            ssh.close()
        except Exception as e:
            self.get_logger().error(f"Failed to send {archive_path}: {e}")
        self.get_logger().info(f"Finished sending {archive_path}")

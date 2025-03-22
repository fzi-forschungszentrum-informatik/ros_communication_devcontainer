#!/usr/bin/env python3

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
# 
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
# 
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.                                                
# -- END LICENSE BLOCK ------------------------------------------------
#
# ---------------------------------------------------------------------
# !\file
#
# \author  Martin Gontscharow <gontscharow@fzi.de>
# \date    2024-11-13
#
#
# ---------------------------------------------------------------------

import os
import sys
import argparse
import datetime
import subprocess
import logging
import threading

try:
    import iperf3
except ImportError:
    print("Please install the iperf3 library: pip install iperf3")
    sys.exit(1)


def setup_logger(log_level="INFO"):
    """
    Configure a logger with the given log level.
    Removes the logger name from the output to save characters.
    """
    logger = logging.getLogger("network_measurement")
    logger.setLevel(getattr(logging, log_level.upper(), logging.INFO))

    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(getattr(logging, log_level.upper(), logging.INFO))

    # Format: only timestamp, log level, and the message
    formatter = logging.Formatter(
        fmt="%(asctime)s [%(levelname)s] - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    console_handler.setFormatter(formatter)

    # Avoid adding multiple handlers if already set
    if not logger.handlers:
        logger.addHandler(console_handler)

    return logger


def measure_ping(host, duration=3, logger=None):
    """
    Run a ping test for 'duration' seconds (using '-w <duration>').
    Returns (avg_ping_ms, packet_loss_percent, jitter_ms, ping_count).
    If something fails, returns (None, None, None, None).

    We'll parse 'X packets transmitted' from the ping output
    so that we know how many pings actually got sent in this duration.
    """
    if logger is None:
        logger = logging.getLogger()

    cmd = ["ping", "-w", str(duration), host]
    logger.debug(f"Running ping command: {' '.join(cmd)}")

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        logger.warning(f"Ping command failed: {result.stderr.strip()}")
        return None, None, None, None

    logger.debug(f"Ping output:\n{result.stdout}")

    output_lines = result.stdout.strip().split("\n")

    # Typically, the last two lines are summary
    transmitted_line = output_lines[-2] if len(output_lines) >= 2 else ""
    rtt_line = output_lines[-1] if len(output_lines) >= 1 else ""

    # Parse how many pings were actually sent
    # e.g. "4 packets transmitted, 4 received, 0% packet loss, time 3005ms"
    ping_count = None
    packet_loss_percent = None
    avg_rtt, jitter_rtt = None, None

    try:
        parts = transmitted_line.split(",")
        # parts might look like:
        #   ["4 packets transmitted", " 4 received", " 0% packet loss", " time 3005ms"]
        if len(parts) >= 1:
            # e.g. parts[0] = "4 packets transmitted"
            transmit_str = parts[0].strip()
            # transmit_str.split() => ["4", "packets", "transmitted"]
            ping_count = int(transmit_str.split()[0])

        if len(parts) >= 3:
            pl_str = parts[2].strip()  # e.g. "0% packet loss"
            packet_loss_percent = float(pl_str.split("%")[0])
    except Exception as e:
        logger.debug(f"Failed to parse 'transmitted' or 'loss': {e}")
        ping_count = None
        packet_loss_percent = None

    # Parse RTT
    try:
        if "=" in rtt_line:
            # e.g. "rtt min/avg/max/mdev = 36.226/60.256/73.029/14.927 ms"
            rtt_values = rtt_line.split("=")[1].strip()
            min_rtt, avg_rtt, max_rtt, jitter_rtt = map(
                lambda x: float(x.replace(" ms", "")),
                rtt_values.split("/")
            )
    except Exception as e:
        logger.debug(f"Failed to parse RTT values: {e}")

    # Round final values
    if avg_rtt is not None:
        avg_rtt = round(avg_rtt, 2)
    if packet_loss_percent is not None:
        packet_loss_percent = round(packet_loss_percent, 2)
    if jitter_rtt is not None:
        jitter_rtt = round(jitter_rtt, 2)

    logger.debug(
        f"Parsed ping -> Count: {ping_count}, AvgPing: {avg_rtt}, Loss: {packet_loss_percent}, Jitter: {jitter_rtt}"
    )
    return avg_rtt, packet_loss_percent, jitter_rtt, ping_count


def measure_upload_bandwidth(host, port=5201, duration=3, logger=None):
    """
    Measure upload bandwidth using iperf3 for 'duration' seconds (minus a small ramp-up).
    Returns the measured upload speed in Mbps or None on failure.
    """
    if logger is None:
        logger = logging.getLogger()

    logger.debug(f"Connecting to iperf3 server at {host}:{port} for {duration}s")

    client = iperf3.Client()
    # The actual iperf test is a bit shorter than 'duration' to reduce ramp-up time confusion
    client.server_hostname = host
    client.port = port
    client.duration = max(duration - 1, 1)  # at least 1 second
    client.reverse = False  # Upload test
    client.timeout = duration + 1  # buffer for timeout

    result = client.run()
    if result.error:
        logger.warning(f"iperf3 failed: {result.error}")
        return None

    upload_speed = round(result.sent_Mbps, 2) if result.sent_Mbps else None
    logger.debug(f"iperf3 upload speed: {upload_speed} Mbps")

    return upload_speed


def perform_measurement(host, port, duration, mode="parallel", logger=None):
    """
    Perform a single measurement (ping + bandwidth).
    If mode == "parallel", run them simultaneously (realistic).
    If mode == "sequential", run them one after another (isolated).
    Returns (avg_ping, packet_loss, jitter, upload_speed, ping_count).
    """
    if logger is None:
        logger = logging.getLogger()

    if mode == "parallel":
        ping_result = {}
        bw_result = {}

        def run_ping():
            avg_ping, packet_loss, jitter, ping_count = measure_ping(
                host, duration=duration, logger=logger
            )
            ping_result["avg_ping"] = avg_ping
            ping_result["loss"] = packet_loss
            ping_result["jitter"] = jitter
            ping_result["count"] = ping_count

        def run_bw():
            speed = measure_upload_bandwidth(
                host, port=port, duration=duration, logger=logger
            )
            bw_result["speed"] = speed

        t_ping = threading.Thread(target=run_ping)
        t_bw = threading.Thread(target=run_bw)

        t_ping.start()
        t_bw.start()
        t_ping.join()
        t_bw.join()

        return (
            ping_result.get("avg_ping"),
            ping_result.get("loss"),
            ping_result.get("jitter"),
            bw_result.get("speed"),
            ping_result.get("count"),
        )
    else:
        avg_ping, packet_loss, jitter, ping_count = measure_ping(
            host, duration=duration, logger=logger
        )
        upload_speed = measure_upload_bandwidth(
            host, port=port, duration=duration, logger=logger
        )
        return (avg_ping, packet_loss, jitter, upload_speed, ping_count)


def continuous_measurement(
    testing_mode="parallel",
    vpn_server="10.8.0.1",
    vpn_port=5201,
    internet_server="iperf.he.net",
    internet_port=5201,
    interval=3,
    use_vpn=False,
    logger=None
):
    """
    Continuously measure network metrics. "interval" is both the ping test duration
    and (approximately) the iperf test duration. No extra wait between loops.
    """
    if logger is None:
        logger = logging.getLogger()

    # Decide which host/port to test
    if use_vpn:
        host = vpn_server
        port = vpn_port
        mode_label = "VPN"
    else:
        host = internet_server
        port = internet_port
        mode_label = "Internet"

    # CSV path
    script_dir = os.path.dirname(os.path.realpath(__file__))
    csv_file = os.path.join(script_dir, "network_measurements.csv")

    # Create CSV header if file doesn't exist
    if not os.path.exists(csv_file):
        with open(csv_file, "w") as f:
            # Add new columns: ping_count, duration_s
            f.write("timestamp,mode,host,avg_ping_ms,packet_loss_percent,jitter_ms,upload_mbps,ping_count,duration_s\n")

    # Initial logging
    logger.info(f"Starting measurements in {mode_label} mode ({testing_mode} test).")
    logger.info(f"Host: {host}:{port}")
    logger.info(f"Measurement duration per test: {interval} seconds")
    logger.info(f"Logging to: {csv_file}")
    logger.info("Press Ctrl+C to stop.\n")

    while True:
        try:
            timestamp = datetime.datetime.now().isoformat()

            # avg_ping_ms, packet_loss_percent, jitter_ms, upload_speed_mbps, pings_sent
            avg_ping_ms, packet_loss_percent, jitter_ms, upload_speed_mbps, pings_sent = perform_measurement(
                host=host,
                port=port,
                duration=interval,
                mode=testing_mode,
                logger=logger
            )

            # Log the result line (without the host, but now includes ping count & interval)
            logger.info(
                f"Mode: {mode_label}, "
                f"Ping: {avg_ping_ms if avg_ping_ms is not None else 'N/A'} ms, "
                f"Loss: {packet_loss_percent if packet_loss_percent is not None else 'N/A'}%, "
                f"Jitter: {jitter_ms if jitter_ms is not None else 'N/A'} ms, "
                f"Upload: {upload_speed_mbps if upload_speed_mbps is not None else 'N/A'} Mbps, "
                f"PingsSent: {pings_sent if pings_sent is not None else 'N/A'}, "
                f"Duration: {interval}s"
            )

            # Write CSV
            with open(csv_file, "a") as f:
                f.write(
                    f"{timestamp},{mode_label},{host},"
                    f"{avg_ping_ms},{packet_loss_percent},{jitter_ms},{upload_speed_mbps},"
                    f"{pings_sent},{interval}\n"
                )

            # Loop immediately for the next measurement
        except KeyboardInterrupt:
            logger.info("Measurement stopped by user.")
            break
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
            break


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Continuously measure ping, packet loss, jitter, and upload bandwidth."
    )
    parser.add_argument(
        "--testing-mode",
        choices=["parallel", "sequential"],
        default="parallel",
        help="Choose 'parallel' (realistic: ping + bandwidth simultaneously) or 'sequential' (isolated)."
    )
    parser.add_argument("--vpn-server", default="10.254.0.33",
                        help="Hostname/IP of the iperf3 server on the VPN.")
    parser.add_argument("--vpn-port", type=int, default=5201,
                        help="Port for the iperf3 VPN server.")
    parser.add_argument("--internet-server", default="speedtest.wtnet.de",
                        help="Hostname/IP of the iperf3 server on the Internet.")
    parser.add_argument("--internet-port", type=int, default=5201,
                        help="Port for the iperf3 Internet server.")
    parser.add_argument("--interval", type=int, default=3,
                        help="Seconds of measurement duration for each test.")
    parser.add_argument("--use-vpn", action="store_true",
                        help="If set, measurements are done against the VPN server.")
    parser.add_argument("--log-level", choices=["DEBUG", "INFO", "WARNING", "ERROR"],
                        default="INFO",
                        help="Set the logging verbosity (default INFO).")

    args = parser.parse_args()
    logger = setup_logger(args.log_level)

    continuous_measurement(
        testing_mode=args.testing_mode,
        vpn_server=args.vpn_server,
        vpn_port=args.vpn_port,
        internet_server=args.internet_server,
        internet_port=args.internet_port,
        interval=args.interval,
        use_vpn=args.use_vpn,
        logger=logger
    )

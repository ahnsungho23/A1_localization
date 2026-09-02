#!/usr/bin/env bash

set -euo pipefail

readonly DEFAULT_HOST_CIDR="192.168.1.100/24"
readonly CONNECTION_NAME="${LIDAR_CONNECTION_NAME:-a1-lidar}"
readonly SYSCTL_FILENAME="90-hesai-lidar.conf"

usage() {
  cat >&2 <<EOF
Usage: sudo $0 <ethernet-interface> [host-cidr]

Configure a dedicated PandarXT host connection.
  ethernet-interface  Required interface name; there is no default
  host-cidr           Optional host address (default: ${DEFAULT_HOST_CIDR})

The NetworkManager connection name defaults to '${CONNECTION_NAME}'.
Set LIDAR_CONNECTION_NAME to use a different name.
EOF
}

validate_ipv4_cidr() {
  local cidr="$1"
  local address prefix octet
  local -a octets

  [[ "${cidr}" == */* ]] || return 1
  address="${cidr%/*}"
  prefix="${cidr##*/}"
  [[ "${prefix}" =~ ^([0-9]|[12][0-9]|3[0-2])$ ]] || return 1

  IFS='.' read -r -a octets <<< "${address}"
  [[ "${#octets[@]}" -eq 4 ]] || return 1

  for octet in "${octets[@]}"; do
    [[ "${octet}" =~ ^[0-9]+$ ]] || return 1
    ((10#${octet} <= 255)) || return 1
  done
}

if (($# < 1 || $# > 2)); then
  usage
  exit 2
fi

readonly INTERFACE="$1"
readonly HOST_CIDR="${2:-${DEFAULT_HOST_CIDR}}"

if ((EUID != 0)); then
  echo "ERROR: This script must be run as root with sudo." >&2
  usage
  exit 1
fi

missing_command=0
for command_name in nmcli ip sysctl install; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "ERROR: Required command not found: ${command_name}" >&2
    missing_command=1
  fi
done
if ((missing_command != 0)); then
  exit 1
fi

if ! ip link show dev "${INTERFACE}" >/dev/null 2>&1; then
  echo "ERROR: Ethernet interface does not exist: ${INTERFACE}" >&2
  exit 1
fi

device_type="$(nmcli -g GENERAL.TYPE device show "${INTERFACE}")"
if [[ "${device_type}" != "ethernet" ]]; then
  echo "ERROR: Interface '${INTERFACE}' has type '${device_type}', not Ethernet." >&2
  exit 1
fi

if ! validate_ipv4_cidr "${HOST_CIDR}"; then
  echo "ERROR: Invalid IPv4 CIDR: ${HOST_CIDR}" >&2
  exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly REPOSITORY_ROOT
readonly SYSCTL_SOURCE="${REPOSITORY_ROOT}/system/${SYSCTL_FILENAME}"
readonly SYSCTL_TARGET="/etc/sysctl.d/${SYSCTL_FILENAME}"

if [[ ! -f "${SYSCTL_SOURCE}" ]]; then
  echo "ERROR: Repository sysctl file not found: ${SYSCTL_SOURCE}" >&2
  exit 1
fi

echo "WARNING: Bringing up '${CONNECTION_NAME}' may briefly disconnect ${INTERFACE}."
echo "WARNING: If your SSH session uses ${INTERFACE}, this operation may terminate it."
echo "Configuring ${INTERFACE} as ${HOST_CIDR}; Wi-Fi remains eligible as the default route."

if nmcli connection show "${CONNECTION_NAME}" >/dev/null 2>&1; then
  connection_type="$(nmcli -g connection.type connection show "${CONNECTION_NAME}")"
  if [[ "${connection_type}" != "802-3-ethernet" ]]; then
    echo "ERROR: Existing connection '${CONNECTION_NAME}' is not Ethernet." >&2
    echo "       Refusing to modify connection type '${connection_type}'." >&2
    exit 1
  fi
  echo "Updating existing NetworkManager connection: ${CONNECTION_NAME}"
else
  echo "Creating NetworkManager connection: ${CONNECTION_NAME}"
  nmcli connection add \
    type ethernet \
    ifname "${INTERFACE}" \
    con-name "${CONNECTION_NAME}"
fi

nmcli connection modify "${CONNECTION_NAME}" \
  connection.interface-name "${INTERFACE}" \
  connection.autoconnect yes \
  ipv4.method manual \
  ipv4.addresses "${HOST_CIDR}" \
  ipv4.gateway "" \
  ipv4.dns "" \
  ipv4.never-default yes \
  ipv6.method disabled

install -m 0644 "${SYSCTL_SOURCE}" "${SYSCTL_TARGET}"
sysctl --load "${SYSCTL_TARGET}"

nmcli connection up id "${CONNECTION_NAME}" ifname "${INTERFACE}"

echo
echo "Configured interface address:"
ip -brief address show dev "${INTERFACE}"
echo
echo "Configured receive-buffer maximum:"
sysctl net.core.rmem_max

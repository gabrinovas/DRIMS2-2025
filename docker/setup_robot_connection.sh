#!/usr/bin/env sh
# Generates $HOME/cyclone_config.xml for CycloneDDS based on the detected subnet/interface.
# POSIX sh compatible.

# ========= CONFIG: set fixed peers per robot here =========
# Space-separated list of peer IPs per robot.
UR5e_LEFT="192.168.1.105"          # UR robots on 192.168.1.x
UR5e_RIGHT="192.168.1.101"         # UR robots on 192.168.1.x
TIAGO_PEERS="10.68.0.1"            # Tiago robots on 10.68.0.x (EDIT as needed)
# ==========================================================

UR5e_PFX="192.168.1."
TIAGO_PFX="10.68.0."

out_file="${HOME}/cyclone_config.xml"

# Find interface that has an IPv4 on a given prefix (e.g., "192.168.1.")
find_iface_for_prefix() {
  pfx="$1"
  # Sample line: "2: enp4s0    inet 192.168.1.42/24 brd ..."
  ip -o -4 addr show 2>/dev/null | awk -v pfx="$pfx" '
    $3 == "inet" {
      # $4 is like "192.168.1.42/24"
      split($4, a, "/");
      if (index(a[1], pfx) == 1) { print $2; exit }
    }
  '
}

# Render peers block as XML lines
# Args: list of IPs
render_peers_xml() {
  for ip in "$@"; do
    [ -n "$ip" ] && printf '        <Peer Address="%s"/>\n' "$ip"
  done
}

# Detect which robot network is present and set variables accordingly
UR5e_IFACE="$(find_iface_for_prefix "$UR5e_PFX")"
TIAGO_IFACE="$(find_iface_for_prefix "$TIAGO_PFX")"

ROBOT=""
IFACE=""
PEERS_XML=""
PEER_LIST="" # For ping

if [ -n "$UR5e_IFACE" ]; then
  ROBOT="UR robot"
  IFACE="$UR5e_IFACE"
  PEERS_XML="$(printf '        <Peer Address="localhost"/>\n%s' "$(render_peers_xml $UR5e_LEFT $UR5e_RIGHT)")"
  PEER_LIST="$UR5e_LEFT $UR5e_RIGHT"   # Peers to ping for connectivity check
elif [ -n "$TIAGO_IFACE" ]; then
  ROBOT="TIAGO robot"
  IFACE="$TIAGO_IFACE"
  PEERS_XML="$(printf '        <Peer Address="localhost"/>\n%s' "$(render_peers_xml $TIAGO_PEERS)")"
  PEER_LIST="$TIAGO_PEERS"   # Peers to ping for connectivity check
else
  echo "❌  No known robot subnet detected (neither 192.168.1.* nor 10.68.0.*)."
  echo "👉  Hint: check 'ip -4 addr' to confirm your assigned addresses."
  exit 1
fi

# Backup existing CycloneDDS config file if it exists
if [ -f "$out_file" ]; then
  cp -f "$out_file" "${out_file}.bak" 2>/dev/null || true
fi

# Write CycloneDDS XML template with placeholders for interface and peers
# Keep ${HOME} literal using single-quoted heredoc.
cat > "$out_file" <<'EOF'
<CycloneDDS>
  <Domain>
    <General>
      <AllowMulticast>false</AllowMulticast>
      <MaxMessageSize>1400B</MaxMessageSize>
      <Interfaces>
        <!-- Note: The name="" must be set according to ip/ifconfig -->
        <NetworkInterface name="__IFACE__" priority="1"/>
        <NetworkInterface name="lo" priority="0"/>
      </Interfaces>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
__PEERS__
      </Peers>
      <MaxAutoParticipantIndex>500</MaxAutoParticipantIndex>
    </Discovery>
    <Internal>
      <Watermarks>
        <WhcHigh>2000kB</WhcHigh>
      </Watermarks>
    </Internal>
    <Tracing>
      <Verbosity>config</Verbosity>
      <OutputFile>${HOME}/.ros/log/cdds.log</OutputFile>
    </Tracing>
  </Domain>
</CycloneDDS>
EOF

# Substitute placeholders (__IFACE__, __PEERS__) in the XML config
# Use awk to safely inject multi-line peers XML.
tmp="$(mktemp 2>/dev/null || echo /tmp/cyclone_cfg.$$)"
awk -v iface="$IFACE" -v peers="$PEERS_XML" '
  {
    if ($0 ~ /__IFACE__/) gsub(/__IFACE__/, iface)
    if ($0 ~ /__PEERS__/) {
      sub(/__PEERS__/, "<<<PEERS_PLACEHOLDER>>>")
    }
    print
  }
' "$out_file" | awk -v peers="$PEERS_XML" '
  {
    if ($0 ~ /<<<PEERS_PLACEHOLDER>>>/) {
      gsub(/<<<PEERS_PLACEHOLDER>>>/, peers)
    }
    print
  }
' > "$tmp" && mv "$tmp" "$out_file"

echo "✅ Detected connection: $ROBOT on interface '$IFACE'."
echo "✅ CycloneDDS config written to: $out_file"

# --- Connectivity check (ping) ---
PING_OK=0
for peer in $PEER_LIST; do
  if ping -c 1 -W 1 -I "$IFACE" -q "$peer" >/dev/null 2>&1; then
    echo "✅ Connectivity: $ROBOT reachable at $peer"
    PING_OK=1
    break
  fi
done
[ "$PING_OK" -eq 0 ] && echo "❌ Connectivity: no peers reachable on $ROBOT network"

# === Check OnRobot Compute Box connectivity ===
echo ""
echo "=== OnRobot Compute Box Check ==="
if [ -n "$UR5e_IFACE" ]; then
    # We're on UR5e network, check OnRobot Compute Box
    ONROBOT_IP="192.168.1.1"
    ONROBOT_PORT="502"
    
    if ping -c 1 -W 1 -I "$IFACE" -q "$ONROBOT_IP" >/dev/null 2>&1; then
        echo "✅ OnRobot Compute Box reachable at $ONROBOT_IP"
        if nc -z -w 2 $ONROBOT_IP $ONROBOT_PORT 2>/dev/null; then
            echo "✅ OnRobot Compute Box Modbus port $ONROBOT_PORT accessible"
            echo "✅ Hardware mode: gripper commands will be sent to real hardware"
            
            # Additional check: verify we can also reach at least one UR controller
            UR_REACHABLE=0
            for ur_peer in $UR5e_LEFT $UR5e_RIGHT; do
                if ping -c 1 -W 1 -I "$IFACE" -q "$ur_peer" >/dev/null 2>&1; then
                    echo "✅ UR controller $ur_peer also reachable"
                    UR_REACHABLE=1
                    break
                fi
            done
            if [ "$UR_REACHABLE" -eq 0 ]; then
                echo "⚠️  UR controllers not reachable, but gripper is available"
            fi
        else
            echo "⚠️  OnRobot Compute Box Modbus port $ONROBOT_PORT not accessible"
            echo "⚠️  Running gripper in simulation mode"
        fi
    else
        echo "❌ OnRobot Compute Box not reachable at $ONROBOT_IP"
        echo "⚠️  Running gripper in simulation mode"
    fi
fi

# === Append to ~/.bashrc if not already present ===
BASHRC="$HOME/.bashrc"
EXPORT_LINE='export CYCLONEDDS_URI=$HOME/cyclone_config.xml'
EXPORT_DDS_TYPE='export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' 

# Add CYCLONEDDS_URI export to .bashrc if not already present
if ! grep -Fxq "$EXPORT_LINE" "$BASHRC"; then
    echo "$EXPORT_LINE" >> "$BASHRC"
    echo "✅ Added export line to $BASHRC"
fi

# If the robot is Tiago, add ROS_DOMAIN_ID=2 and CycloneDDS export if not already present
if [ "$ROBOT" = "TIAGO robot" ]; then
    DOMAIN_LINE='export ROS_DOMAIN_ID=2'
    if ! grep -Fxq "$DOMAIN_LINE" "$BASHRC"; then
        echo "$DOMAIN_LINE" >> "$BASHRC"
        echo "✅ Added ROS_DOMAIN_ID=2 to $BASHRC"
    fi
    if ! grep -Fxq "$EXPORT_DDS_TYPE" "$BASHRC"; then
        echo "$EXPORT_DDS_TYPE" >> "$BASHRC"
        echo "✅ Added CYCLONE to $BASHRC"
    fi
fi

# Source the updated .bashrc to apply changes in the current shell
. "$HOME/.bashrc"
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=0
ENV CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.1.90</NetworkInterfaceAddress></General></Domain></CycloneDDS>'

COPY entrypoint.sh install-deps.sh setup-robot-net.sh setup-robot-eth.sh setup-robot-wifi.sh check-robot.sh source-unitree.sh source-go2-ws.sh cyclonedds.xml cyclonedds-eth.xml go2-bashrc.snippet /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/install-deps.sh \
    /usr/local/bin/setup-robot-net.sh /usr/local/bin/setup-robot-eth.sh /usr/local/bin/setup-robot-wifi.sh \
    /usr/local/bin/check-robot.sh /usr/local/bin/source-unitree.sh /usr/local/bin/source-go2-ws.sh \
    && mkdir -p /etc/cyclonedds \
    && mkdir -p /etc/cyclonedds \
    && cp /usr/local/bin/cyclonedds.xml /etc/cyclonedds/go2.xml \
    && cp /usr/local/bin/cyclonedds-eth.xml /etc/cyclonedds/go2-eth.xml \
    && cat /usr/local/bin/go2-bashrc.snippet >> /root/.bashrc \
    && /usr/local/bin/install-deps.sh nav2

WORKDIR /ws

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]

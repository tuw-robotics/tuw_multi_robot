#!/usr/bin/env python

import yaml
import rospy

from collections import OrderedDict

from tuw_multi_robot_msgs.msg import Station
from tuw_multi_robot_msgs.msg import StationArray

from tuw_multi_robot_srvs.srv import StationManagerControlProtocol
from tuw_multi_robot_srvs.srv import StationManagerControlProtocolRequest
from tuw_multi_robot_srvs.srv import StationManagerControlProtocolResponse
from tuw_multi_robot_srvs.srv import StationManagerStationProtocol
from tuw_multi_robot_srvs.srv import StationManagerStationProtocolRequest
from tuw_multi_robot_srvs.srv import StationManagerStationProtocolResponse


class TuwStationManagerNode:

    def __init__(self):
        # initialize the node
        rospy.init_node('station_manager_node')

        # publisher ready to publish
        self._publisher = rospy.Publisher('/stations', StationArray, queue_size=10)

        # variables to hold update information
        self._publish_on_change = True

        # dictionary to hold stations for work
        self._station_array = OrderedDict()

        # service to communicate with user interface
        self._control_service = \
            rospy.Service('station_manager_control_service', StationManagerControlProtocol, self.control_protocol)

        # service to exchange station information with user interface
        self._station_service = \
            rospy.Service('station_manager_station_service', StationManagerStationProtocol, self.station_protocol)

    def alive(self):
        # keep node alive
        rospy.spin()

    def control_protocol(self, station_manager_control_protocol):
        protocol = {
            StationManagerControlProtocolRequest.LOAD: self.load_data,
            StationManagerControlProtocolRequest.SAVE: self.save_data,
            StationManagerControlProtocolRequest.UPDATE: self.update
        }
        response = protocol[station_manager_control_protocol.request](station_manager_control_protocol)

        if response:
            return response
        else:
            return StationManagerControlProtocolResponse.ERROR

    def station_protocol(self, station_manager_station_protocol):
        protocol = {
            StationManagerStationProtocolRequest.APPEND: self.append_station,
            StationManagerStationProtocolRequest.REMOVE: self.remove_station
        }
        return protocol[station_manager_station_protocol.request](station_manager_station_protocol)

    def append_station(self, station_manager_station_protocol):
        station_name = station_manager_station_protocol.station.name
        station_pose = station_manager_station_protocol.station.pose
        self._station_array[station_name] = station_pose
        self._station_array = OrderedDict(sorted(self._station_array.items()))
        if self._publish_on_change:
            self.publish_data()
        return StationManagerStationProtocolResponse.EXECUTED

    def remove_station(self, station_manager_station_protocol):
        station_name = station_manager_station_protocol.station.name
        if station_name in self._station_array:
            del self._station_array[station_name]
            if self._publish_on_change:
                self.publish_data()
            return StationManagerStationProtocolResponse.EXECUTED
        if station_name not in self._station_array:
            return StationManagerStationProtocolResponse.FAILED

    def load_data(self, station_manager_control_protocol):
        with open(station_manager_control_protocol.addition, "r") as infile:
            self._station_array = yaml.load(infile)
        if self._publish_on_change:
            self.publish_data()
        return StationManagerControlProtocolResponse.EXECUTED

    def save_data(self, station_manager_control_protocol):
        with open(station_manager_control_protocol.addition, "w") as outfile:
            yaml.dump(self._station_array, outfile, default_flow_style=False)
        if self._publish_on_change:
            self.publish_data()
        return StationManagerControlProtocolResponse.EXECUTED

    def update(self, station_manager_control_protocol):
        if str(station_manager_control_protocol.addition) == str(StationManagerControlProtocolRequest.ONCE):
            self.publish_data()
            return StationManagerControlProtocolResponse.EXECUTED

        if str(station_manager_control_protocol.addition) == str(StationManagerControlProtocolRequest.CHANGE):
            self._publish_on_change = not self._publish_on_change
            return StationManagerControlProtocolResponse.EXECUTED

        return StationManagerControlProtocolResponse.ERROR

    def publish_data(self):
        if self._station_array:
            station_array_message = StationArray()
            for name, pose in self._station_array.items():
                station_message = Station()
                station_message.name = name
                station_message.pose = pose
                station_array_message.stations.append(station_message)
            self._publisher.publish(station_array_message)


if __name__ == '__main__':
    node = TuwStationManagerNode()
    node.alive()

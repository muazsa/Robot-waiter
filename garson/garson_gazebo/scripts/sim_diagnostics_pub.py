#!/usr/bin/env python

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header


class SimDiagnostics(object):

    def __init__(self, battery_size="large"):
        """
        This emulates the diagnostics info given by the real kobuki base
        Battery info: https://docs.google.com/document/d/15k7UBnYY_GPmKzQCjzRGCW-4dIP7zl_R_7tWPLM0zKI/edit
        Expected Operating Time: 3/7 hours (small/large battery)

        """
        # Publish rate
        self.rate = rospy.Rate(1)
        # Seconds Max = Num Hours * Minutes in an hour * Seconds in a minute
        if battery_size == "large":
            self.battery_max_seconds = 7 * 60.0 * 60.0
        else:
            self.battery_max_seconds = 3 * 60.0 * 60.0

        self.init_battery_seconds = rospy.get_time()
        self._pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)


    def get_simulated_battery_status(self):
        now_battery_seconds = rospy.get_time()
        working_time = now_battery_seconds - self.init_battery_seconds
        working_time_left = self.battery_max_seconds - working_time
        battery_charge = (working_time_left / self.battery_max_seconds) * 100.0

        if battery_charge < 0:
            battery_charge = 0

        return int(battery_charge)



    def publish_diagnostics_info(self):
        """
        Publishes at a rate of 1Hz ( because thats the rate real kobuki does it )
        For this first version we only publish battery status.
        :return:
        """
        battery_charge = self.get_simulated_battery_status()

        msg = DiagnosticArray()

        info_msg = DiagnosticStatus()

        header = Header()
        header.frame_id = ""
        header.stamp = rospy.Time.now()
        msg.header = header
        msg.status.append(info_msg)

        values = []
        percent_value = KeyValue()
        percent_value.key = "Percent"
        percent_value.value = str(battery_charge)
        # TODO: Add all the other values
        voltage_value = KeyValue()
        voltage_value.key = "Voltage (V)"

        charge_value = KeyValue()
        charge_value.key = "Charge (Ah)"

        capacity_value = KeyValue()
        capacity_value.key = "Capacity (Ah)"

        source_value = KeyValue()
        source_value.key = "Source"

        charging_state_value = KeyValue()
        charging_state_value.key = "Charging State"

        current_value = KeyValue()
        current_value.key = "Current (A)"

        if battery_charge <= 10.0:
            level = DiagnosticStatus.ERROR
            message = "Empty Battery"

        elif 10.0 < battery_charge <= 20.0:
            level = DiagnosticStatus.WARN
            message = "Warning Low Battery"
        elif battery_charge >= 100.0:
            level = DiagnosticStatus.OK
            message = "Maximum"
        else:
            level = DiagnosticStatus.OK
            message = "NormalLevel"

        info_msg.name = "mobile_base_nodelet_manager: Battery"
        info_msg.hardware_id = "Kobuki"
        info_msg.message = message
        info_msg.level = level
        values.append(percent_value)
        values.append(voltage_value)
        values.append(charge_value)
        values.append(capacity_value)
        values.append(source_value)
        values.append(charging_state_value)
        values.append(current_value)

        info_msg.values = values

        msg.status.append(info_msg)

        self._pub.publish(msg)


    def start_loop_diagnostics_info(self):

        while not rospy.is_shutdown():
            self.publish_diagnostics_info()
            self.rate.sleep()



def main():
    rospy.init_node('diagnostics_node', log_level=rospy.WARN)
    load_sensor_obj = SimDiagnostics()
    load_sensor_obj.start_loop_diagnostics_info()

if __name__ == "__main__":
    main()
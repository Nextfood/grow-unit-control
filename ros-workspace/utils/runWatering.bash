rosservice call /growbox_water_valves/pwm 1 100
rosservice call /growbox_water_valves/pwm 2 0
rosservice call /growbox_water_valves/pwm 3 0

rosservice call /growbox_water_pump/relay true

#sleep 1
rosservice call /growbox_water_valves/pwm 2 100
rosservice call /growbox_water_valves/pwm 1 0

#sleep 1
rosservice call /growbox_water_valves/pwm 3 100
rosservice call /growbox_water_valves/pwm 2 0
#sleep 1

rosservice call /growbox_water_pump/relay false

rosservice call /growbox_water_valves/pwm 3 0


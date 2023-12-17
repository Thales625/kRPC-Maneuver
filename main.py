import krpc
from time import sleep
from math import exp, cos, radians
from os import system

from Vector import Vector3

class Maneuver:
    def __init__(self, node=None):
        self.conn = krpc.connect('Manuever')
        self.space_center = self.conn.space_center
        self.vessel = self.space_center.active_vessel
        self.body = self.vessel.orbit.body
        self.body_ref = self.body.reference_frame
        self.surface_ref = self.vessel.surface_reference_frame
        self.flight = self.vessel.flight(self.body_ref)

        # Streams
        self.stream_mass = self.conn.add_stream(getattr, self.vessel, "mass")
        self.stream_av_thrust = self.conn.add_stream(getattr, self.vessel, "available_thrust")
        #self.stream_ut = self.conn.add_stream(getattr, self.space_center, "ut")

        # Params
        min_dv = 0.05

        # Initializing
        node = self.vessel.control.nodes[0]
        if node is None:
            print('node not found')
            exit()
        stream_node_time_to = self.conn.add_stream(getattr, node, 'time_to')
        stream_node_remaining_dv = self.conn.add_stream(getattr, node, 'remaining_delta_v')

        # aim to Node
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.reference_frame = node.reference_frame
        self.vessel.auto_pilot.target_direction = (0, 1, 0)
        #self.vessel.auto_pilot.wait()

        eIsp = 0
        for engine in self.vessel.parts.engines:
            if engine.active:
                eIsp += engine.available_thrust / av_thrust * engine.specific_impulse
        ve = eIsp * 9.82

        while True:
            # get streams
            av_thrust = self.stream_av_thrust()
            mass = self.stream_mass()
            #ut = self.stream_ut()

            dv = Vector3(stream_node_remaining_dv())
            dv_mag = dv.magnitude()
            final_mass = mass * exp(-dv_mag / ve)

            a0 = av_thrust / mass
            a1 = av_thrust / final_mass

            t_burning = (dv_mag/a0 + dv_mag/a1) / 2

            t_to_node = stream_node_time_to()

            t_to_start = t_to_node - t_burning / 2
            #t_to_end = t_to_node + t_burning / 2
            
            #ut_start = ut + t_to_start
            #ut_end = ut + t_to_end

            system("cls")

            if t_to_start <= 0:
                a_eng = av_thrust / mass
                throttle = min(1, max(0, dv_mag / a_eng))

                auto_pilot_error = radians(self.vessel.auto_pilot.error)
                error_factor = cos(auto_pilot_error)

                self.vessel.control.throttle = throttle * error_factor

                if dv_mag <= min_dv:
                    self.vessel.control.throttle = 0
                    break

                print(f'Throttle: {throttle:.2f} | mag: {dv_mag:.2f}')
            else:
                print(f'Burn in: {t_to_start:.2f}')

            sleep(0.05)

if __name__ == '__main__':
    Maneuver()
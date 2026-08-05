import csv


class FlightLogger:

    def __init__(

        self,

        filename="flight_log.csv",

    ):

        self.file = open(

            filename,

            "w",

            newline="",

        )

        self.writer = csv.writer(

            self.file

        )

        self.writer.writerow([

            "Time",

            "X",

            "Y",

            "Z",

            "Roll",

            "Pitch",

            "Yaw",

            "Battery",

            "Motor",

        ])

    def log(

        self,

        t,

        state,

        battery,

        motor,

    ):

        self.writer.writerow([

            t,

            *state[:3],

            *state[6:9],

            battery,

            motor,

        ])

    def close(self):

        self.file.close()

class AutoPilot:

    """
    High-level autopilot wrapper.
    """

    def __init__(

        self,

        planner,

        avoidance,

        controller,

    ):

        self.planner=planner

        self.avoidance=avoidance

        self.controller=controller

    def update(

        self,

        current,

        gps,

        quad,

    ):

        target=self.planner.update(current)

        target=self.avoidance.modify(

            target,

            current,

        )

        return self.controller.update(

            target,

            gps,

            quad,

        )


class PIDError:

    def __init__(self, init_error, init_time):
        self.error = init_error
        self.integrate_error = 0
        self.time = init_time

    def get_pid(self, error, time):

        time_step = time - self.time
        if (time_step == 0):
            raise RuntimeError('time step must be non zero')

        result = (
            (error - self.error) / time_step,
            error,
            (time_step * (error + self.error) / 2) + self.integrate_error
        )

        self.error = error
        self.time = time
        self.integrate_error = result[2]
        return result
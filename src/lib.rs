// Rust implementation of the PID controller from the "Phil's Lab" youtube account.
// https://github.com/pms67/PID

// TODO(bschwind) - * Add some DT time-tracking code
//                  * Use a builder pattern for configuring the PID controller
//                  * Apply this to something real and test it

pub struct PidController {
    kp: f32,
    ki: f32,
    kd: f32,
    tau: f32,
    output_limit: f32,
    integrator_limit: f32,
    dt: f32, // Delta time, in seconds
    integrator: f32,
    prev_error: f32,
    differentiator: f32,
    prev_measurement: f32,
}

impl PidController {
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            tau: 0.0,
            output_limit: 10.0,
            integrator_limit: 5.0,
            dt: 0.01, // Sample time, in seconds
            integrator: 0.0,
            prev_error: 0.0,
            differentiator: 0.0,
            prev_measurement: 0.0,
        }
    }

    pub fn update(&mut self, setpoint: f32, measurement: f32) -> f32 {
        let error = setpoint - measurement;
        let proportional = self.kp * error;

        self.integrator = self.integrator + 0.5 * self.ki * self.dt * (error + self.prev_error);

        // Anti-wind-up integrator clamping
        self.integrator = self.integrator.clamp(-self.integrator_limit, self.integrator_limit);

        self.differentiator = -(2.0 * self.kd * (measurement - self.prev_measurement)
            + (2.0 * self.tau - self.dt) * self.differentiator)
            / (2.0 * self.tau + self.dt);

        let mut output = proportional + self.integrator + self.differentiator;
        output = output.clamp(-self.output_limit, self.output_limit);

        self.prev_error = error;
        self.prev_measurement = measurement;

        output
    }
}

#[cfg(test)]
mod tests {
    use crate::PidController;

    #[test]
    fn basic_pid_test() {
        let mut pid = PidController::new(2.0, 0.5, 0.25);
        let mut test_system = TestSystem::new();

        let mut output = pid.update(25.0, 0.0);

        for i in 0..100 {
            let measurement = test_system.update(output, 0.01);
            output = pid.update(25.0, measurement);

            println!("{}: {}", i, output);
        }
    }

    struct TestSystem {
        output: f32,
        alpha: f32,
    }

    impl TestSystem {
        fn new() -> Self {
            Self { output: 0.0, alpha: 0.02 }
        }

        fn update(&mut self, input: f32, dt: f32) -> f32 {
            self.output = (dt * input + self.output) / (1.0 + self.alpha * dt);
            self.output
        }
    }
}

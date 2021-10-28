// Rust implementation of the PID controller from the "Phil's Lab" youtube account.
// https://github.com/pms67/PID

// TODO(bschwind) - * Add some DT time-tracking code
//                  * Apply this to something real and test it
//                  * Make it no_std compatible

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

#[derive(Debug)]
pub struct PidControllerBuilder {
    kp: f32,
    ki: f32,
    kd: f32,
    tau: f32,
    output_limit: f32,
    integrator_limit: f32,
    dt: f32, // Delta time, in seconds
}

impl Default for PidControllerBuilder {
    fn default() -> Self {
        Self {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            tau: 0.0,
            output_limit: 10.0,
            integrator_limit: 5.0,
            dt: 0.01,
        }
    }
}

impl PidControllerBuilder {
    pub fn build(self) -> PidController {
        PidController {
            kp: self.kp,
            ki: self.ki,
            kd: self.kd,
            tau: self.tau,
            output_limit: self.output_limit,
            integrator_limit: self.integrator_limit,
            dt: self.dt,
            integrator: 0.0,
            prev_error: 0.0,
            differentiator: 0.0,
            prev_measurement: 0.0,
        }
    }

    pub fn p(mut self, p: f32) -> Self {
        self.kp = p;
        self
    }

    pub fn i(mut self, i: f32) -> Self {
        self.ki = i;
        self
    }

    pub fn d(mut self, d: f32) -> Self {
        self.kd = d;
        self
    }

    pub fn dt(mut self, dt: f32) -> Self {
        self.dt = dt;
        self
    }

    pub fn tau(mut self, tau: f32) -> Self {
        self.tau = tau;
        self
    }

    pub fn output_limit(mut self, output_limit: f32) -> Self {
        self.output_limit = output_limit;
        self
    }

    pub fn integrator_limit(mut self, integrator_limit: f32) -> Self {
        self.integrator_limit = integrator_limit;
        self
    }
}

impl PidController {
    pub fn build() -> PidControllerBuilder {
        PidControllerBuilder::default()
    }

    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self::build().p(kp).i(ki).d(kd).build()
    }

    pub fn update(&mut self, setpoint: f32, measurement: f32) -> f32 {
        let error = setpoint - measurement;
        let proportional = self.kp * error;

        self.integrator += 0.5 * self.ki * self.dt * (error + self.prev_error);

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
    use std::ops::{Add, Mul};

    const DT: f32 = 0.01666667;

    #[test]
    fn basic_pid_test() {
        let mut pid = PidController::new(2.0, 0.5, 0.25);
        let mut test_system = TestSystem::new();

        let mut output = pid.update(25.0, 0.0);

        for i in 0..100 {
            let measurement = test_system.update(output, DT);
            output = pid.update(25.0, measurement);

            println!("{}: {}", i, output);
        }
    }

    #[test]
    fn camera_exposure_test() {
        let mut pid = PidController::build().p(8.0).i(0.5).dt(DT).build();

        let mut camera = Camera::new();
        let mut desired_msv = 3.5;

        let mut output;

        for i in 0..100 {
            let msv = camera.msv();
            output = pid.update(desired_msv, msv);
            camera.adjust_exposure(output);

            println!(
                "{}: Target MSV: {}, New MSV: {}, Pid Output: {}, New Exposure: {}",
                i,
                desired_msv,
                msv,
                output,
                camera.exposure()
            );
        }

        desired_msv = 1.3;

        for i in 0..100 {
            let msv = camera.msv();
            output = pid.update(desired_msv, msv);
            camera.adjust_exposure(output);

            println!(
                "{}: Target MSV: {}, New MSV: {}, Pid Output: {}, New Exposure: {}",
                i,
                desired_msv,
                msv,
                output,
                camera.exposure()
            );
        }
    }

    struct Camera {
        exposure: f32,
    }

    impl Camera {
        fn new() -> Self {
            let exposure = 0.1;

            Self { exposure }
        }

        fn adjust_exposure(&mut self, pid_output: f32) {
            self.exposure += pid_output * DT;
            self.exposure = self.exposure.clamp(0.0, 1.0);
        }

        fn msv(&self) -> f32 {
            let noise: f32 = rand::random();
            lerp(0.0, 5.0, self.exposure + (noise * 0.005))
        }

        fn exposure(&self) -> f32 {
            self.exposure
        }
    }

    fn lerp<T>(a: T, b: T, t: f32) -> T
    where
        T: Add<Output = T> + Mul<f32, Output = T>,
    {
        (a * (1.0 - t)) + (b * t)
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

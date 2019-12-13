classdef PID
   properties
        kp = 1;
        ki = 1;
        kd = 0;
        delta = 0.02;
        integral = 0;
        preverr = 0;
        out = 0;
   end
   methods
      function obj = PID(kp, ki, kd, delta)
        obj.kp = kp;
        obj.ki = ki;
        obj.kd = kd;
        obj.delta = delta;
      end
      function out = cal(obj, reference, measured)
        err = reference -  measured;
        obj.integral = obj.integral + err * obj.delta;
        derivative = (err - obj.preverr) / obj.delta;
        out = obj.kp *  err + obj.ki * obj.integral + obj.kd * derivative;
      end
   end
end
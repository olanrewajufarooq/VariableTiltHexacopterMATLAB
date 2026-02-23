classdef SeparatePotential < vt.ctrl.potential.PotentialBase
    properties (Access = private)
        Kp_att
        Kp_pos
    end

    methods
        function obj = SeparatePotential(Kp_att, Kp_pos)
            obj.Kp_att = Kp_att;
            obj.Kp_pos = Kp_pos;
        end

        function Wp = computeWrench(obj, Hd, H)
            R = H(1:3,1:3);
            p = H(1:3,4);
            Rd = Hd(1:3,1:3);
            pd = Hd(1:3,4);

            R_err = Rd' * R;
            e_p = p - pd;

            F = obj.Kp_pos * e_p;
            T = -vt.se3.vee3(0.5 * (obj.Kp_att * R_err - (obj.Kp_att * R_err)'));
            Wp = [T; F];
        end
    end
end

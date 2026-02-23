classdef LieAlgebraPotential < vt.ctrl.potential.PotentialBase
    properties (Access = private)
        Kp
    end

    methods
        function obj = LieAlgebraPotential(Kp)
            obj.Kp = Kp;
        end

        function Wp = computeWrench(obj, Hd, H)
            He = vt.se3.invSE3(Hd) * H;
            zeta = vt.se3.logSE3(He);
            Wp = obj.Kp * zeta;
        end
    end
end

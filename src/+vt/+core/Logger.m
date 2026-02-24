classdef Logger < handle
    properties (Access = private)
        t
        actual
        des
        cmd
        timing
    end

    methods
        function obj = Logger()
            obj.t = [];
            obj.actual = obj.emptyActual();
            obj.des = obj.emptyDesired();
            obj.cmd = obj.emptyCmd();
            obj.timing = obj.emptyTiming();
        end

        function append(obj, t, actual, desired, cmd, timing)
            if nargin < 6 || isempty(timing)
                timing = obj.defaultTiming(t);
            end
            obj.t = [obj.t; t];
            obj.actual = obj.mergeStruct(obj.actual, actual);
            obj.des = obj.mergeStruct(obj.des, desired);
            obj.cmd = obj.mergeStruct(obj.cmd, cmd);
            obj.timing = obj.mergeStruct(obj.timing, timing);
        end

        function logs = finalize(obj)
            logs.t = obj.t;
            logs.actual = obj.actual;
            logs.des = obj.des;
            logs.cmd = obj.cmd;
            logs.timing = obj.timing;
        end
    end

    methods (Access = private)
        function s = mergeStruct(~, s, new)
            fields = fieldnames(new);
            for k = 1:numel(fields)
                f = fields{k};
                s.(f) = [s.(f); new.(f)];
            end
        end

        function s = emptyActual(~)
            s.pos = zeros(0,3);
            s.rpy = zeros(0,3);
            s.linVel = zeros(0,3);
            s.angVel = zeros(0,3);
        end

        function s = emptyDesired(~)
            s.pos = zeros(0,3);
            s.rpy = zeros(0,3);
            s.linVel = zeros(0,3);
            s.angVel = zeros(0,3);
            s.acc6 = zeros(0,6);
        end

        function s = emptyCmd(~)
            s.wrenchF = zeros(0,3);
            s.wrenchT = zeros(0,3);
        end

        function s = emptyTiming(~)
            s.controlTime = zeros(0,1);
            s.adaptationTime = zeros(0,1);
        end

        function s = defaultTiming(~, t)
            s.controlTime = t;
            s.adaptationTime = t;
        end
    end
end

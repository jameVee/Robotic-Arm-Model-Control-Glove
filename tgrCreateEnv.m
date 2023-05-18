function env = tgrCreateEnv(parts, colors)
% Based on exampleCommandBuildWorld() command
env = struct();

% Construct workstation สร้างโต๊ะ
bench = collisionBox(0.5, 0.7, 0.05);
beltL = collisionBox(1.3, 0.4, 0.05);
beltR = collisionBox(1.3, 0.4, 0.05);

% ย้ายโต๊ะ
TBench = trvec2tform([0.4 0 0.2]);
TBeltL = trvec2tform([0 -0.6 0.2]);
TBeltR = trvec2tform([0 0.6 0.2]);

bench.Pose = TBench;
beltL.Pose = TBeltL;
beltR.Pose = TBeltR;
    
env.Station = {bench, beltL, beltR};

% Construct parts
env.Parts = cell(1, length(parts));
for i = 1:length(parts)
    box = collisionBox(0.06, 0.06, 0.1); % หน่วยเมตร
    TBox = trvec2tform(parts{i});
    box.Pose = TBox;
    part.mesh = box;
    part.color = colors(i);
    part.centerPoint = tform2trvec(part.mesh.Pose);
    part.plot = [];
    env.Parts{i} = part;
end
end
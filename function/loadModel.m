function robot = loadModel(urdf_path)
% load_model Loads a URDF model into a RigidBodyTree object
%   gofa_robot = load_model(urdf_path)
%   - urdf_path : path to the .urdf file
%   - returns a RigidBodyTree object with configured data format and gravity

    robot = importrobot(urdf_path);
    robot.DataFormat = 'row';
    robot.Gravity = [0 0 -9.81];

end
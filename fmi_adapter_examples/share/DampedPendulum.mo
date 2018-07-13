model DampedPendulum
  inner Modelica.Mechanics.MultiBody.World world(label2 = "z", n = {0, 0, -1})  annotation(
    Placement(visible = true, transformation(origin = {-92, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  output Modelica.Mechanics.MultiBody.Joints.Revolute revolute1(n = {0, 1, 0},phi(start = 1.5708), useAxisFlange = true) annotation(
    Placement(visible = true, transformation(origin = {-26, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body body1(m = 1, r_CM = {0, 0, -1}) annotation(
    Placement(visible = true, transformation(origin = {24, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Damper damper1(d = 0.1) annotation(
    Placement(visible = true, transformation(origin = {-30, 74}, extent = {{14, -14}, {-14, 14}}, rotation = 0)));
equation
  connect(revolute1.support, damper1.flange_b) annotation(
    Line(points = {{-32, 14}, {-32, 45}, {-44, 45}, {-44, 74}}));
  connect(damper1.flange_a, revolute1.axis) annotation(
    Line(points = {{-16, 74}, {-16, 45}, {-26, 45}, {-26, 14}}));
  connect(world.frame_b, revolute1.frame_a) annotation(
    Line(points = {{-82, 4}, {-36, 4}, {-36, 4}, {-36, 4}}, color = {95, 95, 95}));
  connect(revolute1.frame_b, body1.frame_a) annotation(
    Line(points = {{-16, 4}, {14, 4}, {14, 4}, {14, 4}}, color = {95, 95, 95}));
  annotation(
    uses(Modelica(version = "3.2.2")));
end DampedPendulum;
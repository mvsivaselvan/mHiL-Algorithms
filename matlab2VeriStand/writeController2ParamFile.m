function writeController2ParamFile(controller, filename)

% controller is a discrete-time state space system

fid = fopen(filename,'w');

Ac = controller.A'; % transpose get it to row major form
Ac = Ac(:);
str = matlab2VSparam(Ac);
fprintf(fid,'%s\n',str);

Bc = controller.B';
Bc = Bc(:);
str = matlab2VSparam(Bc);
fprintf(fid,'%s\n',str);

Cc = controller.C';
Cc = Cc(:);
str = matlab2VSparam(Cc);
fprintf(fid,'%s\n',str);

Dc = controller.D';
Dc = Dc(:);
str = matlab2VSparam(Dc);
fprintf(fid,'%s\n',str);

fclose(fid);

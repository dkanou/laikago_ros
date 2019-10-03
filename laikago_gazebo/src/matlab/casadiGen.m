import casadi.*

q = SX.sym('q', 12, 1);

[p_feet,J_feet] = kinBodyFeet(q);

f_kinBodyFeet = Function('kinBodyFeet', {q}, {p_feet, J_feet},...
    {'q'},{'p_feet','J_feet'});

file_folder = getFolderPath();
            cd([file_folder, '/gen']);
f_kinBodyFeet.generate('kinBodyFeet.c',...
    struct('casadi_real', 'double',...
        'casadi_int', 'long long int'));
system(['gcc -fPIC -shared -O3 kinBodyFeet.c -o ',...
                file_folder, '/gen/kinBodyFeet.so']);
            
cd(file_folder);
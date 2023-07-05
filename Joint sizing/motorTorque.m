%% MIT License
% 
% Copyright (C) 2022  Ilya Brodoline - AMU ISM, Marseille, (France).
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% contact: ilya.brodoline@univ-amu.fr
%%

function outputTorque = motorTorque(app)
            alpha = app.kalpha;
            P = (app.kp)*1e-3;
            od = (app.kod)*1e-2;
            width = (app.kwidth)*1e-2;
            height = (app.kheight)*1e-2;
            cv = app.kcv;
            
            outputTorque = zeros(1,length(alpha));
            
        for i=1:length(alpha)
            Alpha = alpha(i)*pi/180;
            
            beta = atan2((od*sin(Alpha)-height),(od*cos(Alpha)+width));
            
            outputTorque(i) = -P*cv / (2*pi*od*sin(beta-Alpha));
        end
        
            lb = (app.kscale(1)+1) / 0.2;
            lh = (app.kscale(2)) / 0.2;
            app.MaxtorquemNmEditField.Value = max(outputTorque(lb:lh))*1e3;
            
        end
        
       
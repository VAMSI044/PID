#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

    PID::Kp=Kp;
    PID::Ki=Ki;
    PID::Kd=Kd;

    p.push_back(Kp);
    p.push_back(Kd);
    p.push_back(Ki);

    d.push_back(0.5);
    d.push_back(0.5);
    d.push_back(0.5);

    p_error = 0;
    i_error = 0;
    d_error = 0;

    twiddle_for_steps = 200;
    best_error = 0;
}


void PID::UpdateError(double cte) {
    
    double t_error;
    
    if ( steps == 0){
        prev_cte = cte;
        steps +=1;
        
    }
    p_error =  cte;
    d_error = cte - prev_cte;
    i_error += cte;

    t_error = TotalError();
    

    if ( steps % twiddle_for_steps == 0 )
    {   
        best_error = t_error;
        
        Kp = p[0];
        Kd = p[1];
        Ki = p[2];
        for( unsigned int i = 0 ; i <= p.size() ; i++ )
        {
            p[i] += d[i];
            if ( t_error < best_error)
            {
                best_error = t_error;
                d[i] *= 1.1;
            }
            else
            {
                p[i] -= 2 * d[i];
 
                t_error = TotalError();

                if ( t_error < best_error)
                {
                    best_error = t_error;
                    d[i] *= 1.1;    
                }
                else
                {
                    d[i] *= 0.9;
                }
            }
        }
          
        if (p[0] < Kp){
            if (p[0]< 0){
                p[0] = (p[0]*0.01) + Kp;
            }            
            else{
                p[0] = (p[0]*0.01) - Kp;
            }
        }
        else{
            if (p[0]<0){
                p[0] = (p[0]*0.01) - Kp;
            }            
            else{
                p[0] = (p[0]*0.01) + Kp;
            }
        }
        

        if (p[1] < Kd){
            if (p[1]<0){
                p[1] = (p[1]*0.01) + Kd;
            }            
            else{
                p[1] = (p[1]*0.01) - Kd;
            }
        }
        else{
            if (p[1]<0){
                p[1] = (p[1]*0.01) - Kd;
            }            
            else{
                p[1] = (p[1]*0.01) + Kd;
            }
        }


        if (p[2] < Ki){
            if (p[2]<0){
                p[2] = (p[2]*0.01) + Ki;
            }            
            else{
                p[2] = (p[2]*0.01) - Ki;
            }
        }
        else{
            if (p[2]<0){
                p[2] = (p[2]*0.01) - Ki;
            }            
            else{
                p[2] = (p[2]*0.01) + Ki;
            }
        }
        
        p[1] = (p[1]*0.01) + Kd;
        p[2] = (p[2]*0.01) + Ki;
        t_error = TotalError();
           
    }

    prev_cte = cte;
    steps+=1;
}

double PID::TotalError() {

    return ( -(p[0] * p_error) - (p[1] * d_error) - (p[2] * i_error) );
}


// Control Rudder File that Davi Used 
// Updated by JL 5/8/18

//defining the max and min limits for the boat's orientation (angle)
float saturador(float sensor) {
  if (sensor > 180) {
    sensor = sensor - 360;
  }
  if (sensor < -180) {
    sensor = sensor + 360;
  }
  return sensor;
}


// defining the max and min limits for the rudder angles 
float saturador_rudder(float sensor) {
  if (sensor > 90) {
    sensor = 90;
  }
  if (sensor < -90) {
    sensor = -90;
  }
  return sensor;
}

//  saturator_actuator_color??
int saturador_atuador_cor(float sensor) {
  float id = (-sensor + 120)/30;
  int aux = (int)id;
  float decpart = id - aux;
  int idreturn = 1;
  if (decpart <= 0.5){
    idreturn = id;
  } else{
    idreturn = id + 1;
  }
  return idreturn;
}

// control rudder
// called once in the sailboatMainCode_2017 line: 353 
void controle_rudder() {
  // sp - angle that the sailboat needs to travel to reach waypoint
  sp = saturador(sp);
  // heading - actual angle found from the compass 
  erroAtual = sp - heading;
  // to make sure the angles do not pass 
  erroAtual = saturador(erroAtual);

  // stores current angle as the previous angle
  // angle_rudder_previous 
  angulo_rudder_ant = angulo_rudder;

  //from his rudder control laws 
  angulo_rudder = 90 * (erroAtual/180);

  // error = desired angle - actual angle 
  erroAtual = angulo_rudder;

  // apply PI theory 
  angulo_rudder = P() + I();
  angulo_rudder = saturador_rudder(angulo_rudder);
  //angulo_rudder_sig = rudder_signal(angulo_rudder);

  // not sure what this does yet...
  angulo_rudder_sig = saturador_atuador_cor(angulo_rudder);
  // this is the final output from the rudder controller 
  // Outputs: rudder angle 
  Serial1.print(angulo_rudder_sig);
}

float P()
{
  return Kp * erroAtual;
}

float I()
{
  if ((Ianterior > 0 && erroAtual < 0) || (Ianterior < 0 && erroAtual > 0))
  {
    // Iprevious                 current Error    time cycle 
    Ianterior = Ianterior + Ki * erroAtual * 50 * tempo_do_ciclo;
  }
  else
  {
                                            // time cycle
    Ianterior = Ianterior + Ki * erroAtual * tempo_do_ciclo;
  }
  return Ianterior;
}
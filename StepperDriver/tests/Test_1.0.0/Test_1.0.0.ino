void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("init");
  Serial.println(0);
  pinMode(12, OUTPUT);

  const uint16_t t_time = 1000;
  const uint16_t min_t = 2;
  const uint16_t max_t = 100;
  uint16_t t_steps = 0;

  for (uint16_t x = 0; x < t_time;)
  {
    // Serial.println(lerp(max_t, min_t, quad((float)((float)x / (float)t_time))));
    x += lerp(max_t, min_t, easeout((float)((float)x / (float)t_time)));
    t_steps++;
  }
  Serial.println(t_steps);

  uint32_t t = 0;
  t_steps = 0;
  while (t < t_time)
  {
    // Serial.println(lerp(max_t, min_t, easeout((float)t / (float)t_time)));
    delay(lerp(max_t, min_t, easeout((float)t / (float)t_time)));
    t += lerp(max_t, min_t, easeout((float)t / (float)t_time));
    t_steps++;
    digitalWrite(12, HIGH);
    delay(1);
    digitalWrite(12, LOW);
  }
  Serial.println(t_steps);

  while (1)
  {
    delay(min_t);
    digitalWrite(12, HIGH);
    delay(1);
    digitalWrite(12, LOW);
  }
}

void loop()
{
}

float quad(float time)
{
  if (time < .5)
    return easein(time * 2.0) / 2.0;
  time -= .5;
  return (easeout(time * 2.0) / 2.0) + 0.5;
}
float easein(float time)
{
  return time * time;
}
float easeout(float time)
{
  return -1.0 * time * (time - 2.0);
}

float Q_quad(float time)
{
  if (time < .5)
    return easeout(time * 2.0) / 2.0;
  time -= .5;
  return (easein(time * 2.0) / 2.0) + 0.5;
}

int16_t lerp(int16_t point_a, int16_t point_b, float time)
{
  return ((((int16_t)point_b - (int16_t)point_a) * (float)time) + (int16_t)point_a);
}

float perl(int16_t point_a, int16_t point_b, int16_t val)
{
  return (float)((val - point_a) / (float)(point_b - point_a));
}
#include "ackerman.h"

AckermanModel::AckermanModel(float wheelBase)
{
    this->wheelBase = wheelBase;
    this->v_x = 0;
    this->v_y = 0;
    this->v_theta = 0;
}

void AckermanModel::UpdateParaemters(float velocity, float steering)
{
    this->v_x = velocity * cos(steering);
    this->v_y = velocity * sin(steering);
    this->v_theta = (velocity / this->wheelBase) * tan(v_theta);
}

float* AckermanModel::getPoints(float delta_time)
{
    float *points = new float[3];
    points[0] = v_x * delta_time;
    points[1] = v_y * delta_time;
    points[2] = v_theta * delta_time;
    return points;
}


int main()
{
    AckermanModel test(1);
    test.UpdateParaemters(5,0);
    float *res = test.getPoints(5);
    std::cout << res[0] << ',' << res[1] << ',' << res[2] << std::endl;
    return 0;
}
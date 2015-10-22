#pragma once

namespace tano
{
  //------------------------------------------------------------------------------
  struct RandomUniform
  {
    float Next(float minValue = 0.f, float maxValue = 1.f);
    int _idx = 0;
  };

  //------------------------------------------------------------------------------
  struct RandomInt
  {
    int Next();
    int _idx = 0;
  };

  //------------------------------------------------------------------------------
  // NB: Don't read too much into the gaussian distribution names. I'm pretty sure
  // their std devs don't match to what's given :)
  struct RandomGaussBase
  {
    RandomGaussBase(float* table);
    float Next(float mean, float variance);

    float* _table;
    int _idx = 0;
  };

  //------------------------------------------------------------------------------
  struct RandomGauss01 : public RandomGaussBase
  {
    RandomGauss01();
  };

  //------------------------------------------------------------------------------
  struct RandomGauss10 : public RandomGaussBase
  {
    RandomGauss10();
  };

  //------------------------------------------------------------------------------
  struct RandomGauss50 : public RandomGaussBase
  {
    RandomGauss50();
  };

  //------------------------------------------------------------------------------
  struct RandomGauss150 : public RandomGaussBase
  {
    RandomGauss150();
  };

  void ShowRandomDistribution();
}
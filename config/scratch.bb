namespace intro
{
    float explodeTime = 22.0;
    float moveSpeed = 15.0;
    float rotSpeed = 3.0;

    vec3 lineParams = { 5.0, 1.25, 500 };

    vec3 text0Pos = { 0, 300, 0 };
    float text0Scale = 0.75;
    float text0FadeInStart = 6;
    float text0FadeInEnd = 12;
    float text0FadeOutStart = 20;
    float text0FadeOutEnd = 22;

    vec3 text1Pos = { 0, 0, 0 };
    float text1Scale = 0.75;
    float text1FadeInStart = 8;
    float text1FadeInEnd = 14;
    float text1FadeOutStart = 20;
    float text1FadeOutEnd = 22;

    vec3 text2Pos = { 0, -300, 0 };
    float text2Scale = 0.75;
    float text2FadeInStart = 10;
    float text2FadeInEnd = 16;
    float text2FadeOutStart = 20;
    float text2FadeOutEnd = 22;

    float maxStrength = 5000;
}

namespace landscape
{
    float landscapeClearance = 15;
    float gravity = -5;
    float pushForce = 2;
}

namespace split
{
    float speedMean = 0.5;
    float speedVar = 0.25;
}

namespace plexus
{
    vec3 lineParams = { 5.0, 0.25, 150 };
    float rotXDivisor = 2000;
    float rotYDivisor = 300;
    float strength = 1000;
}

namespace tunnel
{
    vec3 lineParams = { 5.0, 1.25, 1500 };
    float radius = 200;
    float speed = 5;
    int segments = 20;

    vec2 cameraParams = { 100, 100 };

    float dirScale = 0.1; 
}

namespace fluid
{
    int blobSize = 20;
    float timeScale = 1;
    float diffuseScale = 0.1;
    float diffuseStrength = 1;
    float velocityStrength = 1;
    float diff = 0.0001;
    float visc = 0.0001;

    float changeProb = 0.55;

    float speedMean = 0.15;
    float speedVariance = 0.05;
    int particlesPerSegment = 250;

    float updateSpeed = 1.5;
}

namespace particle_trail
{
    vec3 lineParams = { 10.0, 1.0, 250 };
    vec4 upper = {0, 0, 0, 0};
    vec4 lower = {0.03, 0, 0, 0};
}

namespace credits
{
    vec4 upper = {0, 0, 0, 0};
    vec4 lower = {0, 0, 0.003, 0};

    int numParticles = 5000;

    float particleHeight = 50;
    float particleHeightVar = 55;

    float particleSpeed = 55;
    float particleSpeedVar = 55;

    float particleAngleSpeed = 5;
    float particleAngleSpeedVar = 2;

    float particleFadeSpeed = 2;
    float particleFadeSpeedVar = 1;

    float angleVar = 2;

    float waveWidth = 200;
}

namespace radial
{
    float radius_mult = 0.1;
    float radius_scale = 0.01;
}
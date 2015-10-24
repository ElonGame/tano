namespace intro
{
    float explodeTime = 22.0;
    float moveSpeed = 15.0;
    float rotSpeed = 3.0;

    vec3 lineParams = { 5.0, 2.25, 1500 };

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

    float textRight = 0.97;
    float textSize = 0.03;

    float newText0Pos = 0.84;
    float newText1Pos = 0.90;
    float newText2Pos = 0.96;

    expr distortBrightness = "lfade_in(9.3, 14, t)";
    expr text0Brightness = "1 + max(pulselfade(14.5, 15.0, 16, t), pulselfade(20, 20.5, 21.5, t))";
    expr text1Brightness = "1 + max(pulselfade(15.5, 16.0, 17, t), pulselfade(20, 20.5, 21.5, t))";
    expr text2Brightness = "1 + max(pulselfade(16.5, 17.0, 18, t), pulselfade(20, 20.5, 21.5, t))";

    float maxStrength = 5000;
}

namespace landscape
{
    float clearance = 10;
    float pushForce = 2;

    float sepScale = 0.2;
    float slowingDistance = 5;

    float spacing = 30;
    float spacingForce = 0.2;

    vec3 sunDir = {0.5, 0, 1};
}

namespace split
{
    float speedMean = 0.5;
    float speedVar = 0.25;

    float camOffsetY = -50;
    float camSpeedY = 0;
    float camStartRadius = 20;
    float camEndRadius = 50;
    float camRotSpeed = 0.2425;
    float camRotTime = 30;
    float camTargetJitter = 5;
    float camTargetJitterSpeed = 0.4;
}

namespace plexus
{
    vec3 lineParams = { 15.0, 0.50, 150 };
    float rotXDivisor = 10000;
    float rotYDivisor = 6000;
    float strength = 1000;

    float base = 1000;
    float scale = 750;

    vec3 plexusCamPos = { -400, 400, -600 };
    vec3 plexusCamLookAt = { -400, 300, 0 };

    vec3 greetsCamPos = { 0, 0, -50 };
    vec3 greetsCamLookAt = { 0, 0, 0 };
}

namespace tunnel
{
    vec3 lineParams = { 7.0, 2.25, 1500 };
    float radius = 200;
    float speed = 5;
    int segments = 50;

    vec2 cameraParams = { 100, 100 };

    vec3 gravity = { 0, -10, 0 };
    float damping = 0.001;
    float windForce = 2;
    float forceSpeed = 0.5;

    float dirScale = 0.1;

    float ofsScale = 23.12;
    float lenScale = 0.01;
    float rScale = 5.233;
    float rProb = 0.5;

    float cloudParticleSize = 15;
    vec4 cloudParticleColor = {1, 1, 1, 1};

    vec3 snakeLineParams = { 5.0, 2.25, 1500 };
    float snakeParticleSize = 10;
    vec4 snakeParticleColor = {0.8, 0.8, 1, 1};
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

    float textRight = 0.75;
    float textSize = 0.045;

    float textLeft = 0.025;

    float textPos0 = 0.75;
    float textPos1 = 0.85;
}

namespace radial
{
    float radius_mult = 0.1;
    float radius_scale = 0.01;
}
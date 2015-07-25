namespace intro
{
    float explodeTime = 22.0;
    float moveSpeed = 15.0;
    float rotSpeed = 3.0;

    vec3 lineParams = { 5.0, 0.25, 150 };
    vec3 text0pos = { -600, 400, 0 };
    float text0Scale = 0.5;
    float text0FadeInStart = 2.7;
    float text0FadeInEnd = 5;
    float text0FadeOutStart = 7;
    float text0FadeOutEnd = 8.3;

    vec3 text1pos = { 0, 50, 0 };
    float text1Scale = 1;
    float text1FadeInStart = 8.3;
    float text1FadeInEnd = 11;
    float text1FadeOutStart = 14;
    float text1FadeOutEnd = 16;

    vec3 text2pos = { 600, -500, 0 };
    float text2Scale = 0.5;
    float text2FadeInStart = 16.6;
    float text2FadeInEnd = 18.9;
    float text2FadeOutStart = 23;
    float text2FadeOutEnd = 25;

    float maxStrength = 5000;
}

namespace landscape
{
    float landscapeClearance = 30;
    float gravity = -5;
    float pushForce = 20;
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
    float radius = 150;
    float speed = 5;
    int segments = 20;

    vec2 cameraParams = { 100, 100 };

    float dirScale = 0.1; 
}

namespace fluid
{
    int blobSize = 11;
    float timeScale = 10;
    float diffuseScale = 1.25;
    float diffuseStrength = 5;
    float velocityStrength = 10;
    float diff = 0.001;
    float visc = 2;
}

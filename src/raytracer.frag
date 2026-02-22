#version 330 core
out vec4 FragColor;

uniform vec3 cameraPos;
uniform vec3 cameraDir;   
uniform vec3 cameraUp;    
uniform vec3 cameraRight;
uniform float fov;        
uniform float aspect;     
uniform vec2 resolution;  

uniform samplerBuffer sceneVertices; 
uniform int numTriangles;            

uniform mat4 u_model;

uniform int u_mirrorMode; //0 = Matte Shadows, 1 = Mirror
uniform int u_wallsEnabled;
uniform int u_lightBackground;

uniform vec3 u_light1Pos;
uniform vec3 u_light2Pos;
uniform vec3 u_light1Color;
uniform vec3 u_light2Color;

const float FLT_MAX = 1e30;
const float EPSILON = 0.001;
const int MAX_DEPTH = 12; 

struct Ray { vec3 origin; vec3 dir; };
struct HitRecord { float t; vec3 normal; bool hit; bool frontFace; int type; };

// Environment 
vec3 getEnvironment(vec3 dir) {
    float t = 0.5 * (dir.y + 1.0);
    
    if (u_lightBackground == 1) {
        return vec3(0.7, 0.85, 1.0); 
    } else {
        return mix(vec3(0.05, 0.05, 0.1), vec3(0.2, 0.2, 0.3), t); 
    }
}

// Intersection
HitRecord traceScene(Ray r) {
    HitRecord closest;
    closest.t = FLT_MAX;
    closest.hit = false;
    closest.normal = vec3(0.0);
    closest.frontFace = true;
    closest.type = -1;

    // Floor
    float tPlane = -(r.origin.y + 1.0) / r.dir.y;
    if (tPlane > EPSILON && tPlane < closest.t) {
        closest.t = tPlane;
        closest.hit = true;
        closest.normal = vec3(0.0, 1.0, 0.0);
        closest.frontFace = r.dir.y < 0.0;
        closest.type = 1; 
    }
    // Walls (in an hexagonal disposal)
    if (u_wallsEnabled == 1) {
        float R = 8.0; // Distance of the walls from the center. Adjust as needed!
        vec3 wallNormals[3];
        wallNormals[0] = vec3(0.0, 0.0, 1.0);                  // Back wall
        wallNormals[1] = vec3(0.866025, 0.0, 0.5);             // Back-left (sqrt(3)/2 for X)
        wallNormals[2] = vec3(-0.866025, 0.0, 0.5);            // Back-right

        for(int i = 0; i < 3; i++) {
            vec3 n = wallNormals[i];
            // Plane equation intersection
            float tWall = (-R - dot(n, r.origin)) / dot(n, r.dir);
            if (tWall > EPSILON && tWall < closest.t) {
                closest.t = tWall;
                closest.hit = true;
                closest.normal = n;
                closest.frontFace = dot(r.dir, n) < 0.0;
                closest.type = 1; // Assigning type 1 so it shares the floor's properties
            }
        }
    }

    // Diamond
    mat4 model = u_model; 
    if (model[0].x == 0.0 && model[1].y == 0.0 && model[2].z == 0.0) model = mat4(1.0);

    for (int i = 0; i < numTriangles; i++) {
        int baseIndex = i * 3;
        vec3 rawV0 = texelFetch(sceneVertices, baseIndex + 0).rgb;
        vec3 rawV1 = texelFetch(sceneVertices, baseIndex + 1).rgb;
        vec3 rawV2 = texelFetch(sceneVertices, baseIndex + 2).rgb;

        vec3 v0 = (model * vec4(rawV0, 1.0)).xyz;
        vec3 v1 = (model * vec4(rawV1, 1.0)).xyz;
        vec3 v2 = (model * vec4(rawV2, 1.0)).xyz;

        vec3 edge1 = v1 - v0;
        vec3 edge2 = v2 - v0;
        vec3 h = cross(r.dir, edge2);
        float a = dot(edge1, h);

        if (a > -0.00001 && a < 0.00001) continue;
        float f = 1.0 / a;
        vec3 s = r.origin - v0;
        float u = f * dot(s, h);
        if (u < 0.0 || u > 1.0) continue;
        vec3 q = cross(s, edge1);
        float v = f * dot(r.dir, q);
        if (v < 0.0 || u + v > 1.0) continue;
        float t = f * dot(edge2, q);
        
        if (t > EPSILON && t < closest.t) {
            closest.t = t;
            closest.hit = true;
            closest.type = 0; 
            vec3 n = normalize(cross(edge1, edge2));
            if (dot(r.dir, n) > 0.0) { closest.normal = -n; closest.frontFace = false; } 
            else { closest.normal = n; closest.frontFace = true; }
        }
    }
    return closest;
}

float calculateShadow(vec3 origin, vec3 lightDir, float maxDist) {
    Ray shadowRay;
    shadowRay.origin = origin + lightDir * EPSILON;
    shadowRay.dir = lightDir;
    HitRecord hit = traceScene(shadowRay);
    if (hit.hit && hit.t < maxDist) return 0.2; // shadow darkness
    return 1.0; 
}

vec3 traceWhitted(Ray r, float ior) {
    vec3 finalColor = vec3(0.0);
    vec3 throughput = vec3(1.0);
    
    for (int depth = 0; depth < MAX_DEPTH; depth++) {
        HitRecord rec = traceScene(r);
        
        if (!rec.hit) {
            finalColor += throughput * getEnvironment(r.dir);
            break;
        }

        // if we reached the last bounce and hit a face, we sample the environment as a fallback, instead of displaying black
        if (depth == MAX_DEPTH - 1) {
            finalColor += throughput * getEnvironment(reflect(r.dir, rec.normal));
            break;
        }
        
        vec3 hitPoint = r.origin + r.dir * rec.t;
        vec3 N = rec.normal;

        // Floor and walls
        if (rec.type == 1) {
            //float pattern = mod(floor(hitPoint.x) + floor(hitPoint.z), 2.0); //floor checkered pattern
            float pattern;
            
            //if the normal is pointing mostly up, it's the floor
            if (abs(N.y) > 0.5) {
                pattern = mod(floor(hitPoint.x) + floor(hitPoint.z), 2.0);
            } 
            // otherwise, it's a vertical wall
            else {
                // Calculate a horizontal tangent vector to wrap the texture without stretching
                float tangentU = N.z * hitPoint.x - N.x * hitPoint.z;
                pattern = mod(floor(tangentU) + floor(hitPoint.y), 2.0);
            }

            // Light 1 (Red)
            vec3 L1 = normalize(u_light1Pos - hitPoint);
            float dist1 = length(u_light1Pos - hitPoint);
            float shadow1 = calculateShadow(hitPoint, L1, dist1);
            float diff1 = max(dot(N, L1), 0.0);

            // Light 2 (Blue)
            vec3 L2 = normalize(u_light2Pos - hitPoint);
            float dist2 = length(u_light2Pos - hitPoint);
            float shadow2 = calculateShadow(hitPoint, L2, dist2);
            float diff2 = max(dot(N, L2), 0.0);

            // Combine both lights!
            vec3 lightEnergy = (u_light1Color * diff1 * shadow1) + (u_light2Color * diff2 * shadow2);

            // Mirror mode
            if (u_mirrorMode == 1) {
                vec3 albedo = pattern > 0.5 ? vec3(0.05) : vec3(0.1); 
                finalColor += throughput * (albedo * 0.2 + albedo * lightEnergy) * 0.2;
                throughput *= vec3(0.8); 
                r.origin = hitPoint + N * EPSILON;
                r.dir = reflect(r.dir, N);
                continue; 
            }
            // Matte mode
            else {
                vec3 albedo = pattern > 0.5 ? vec3(0.9) : vec3(0.6); 
                finalColor += throughput * (albedo * 0.2 + albedo * lightEnergy);
                break; 
            }
        }
        
        // Diamond
        if (rec.type == 0) {
            vec3 hitPoint = r.origin + r.dir * rec.t;
            vec3 N = rec.normal;

        // Surface shine
        vec3 viewDir = normalize(cameraPos - hitPoint);
            
            // Diamond highlight from light 1
            vec3 L1 = normalize(u_light1Pos - hitPoint);
            vec3 half1 = normalize(L1 + viewDir);
            float spec1 = pow(max(dot(N, half1), 0.0), 100.0);
            
            // Diamond highlight from light 2
            vec3 L2 = normalize(u_light2Pos - hitPoint);
            vec3 half2 = normalize(L2 + viewDir);
            float spec2 = pow(max(dot(N, half2), 0.0), 100.0);

            // Add the colored highlights to the final color
            finalColor += (u_light1Color * spec1 + u_light2Color * spec2) * 0.8;


            float eta = rec.frontFace ? 1.0 / ior : ior;
            vec3 refracted = refract(r.dir, N, eta);
            
            float f0 = pow((1.0 - ior) / (1.0 + ior), 2.0);
            float fresnelTerm = f0 + (1.0 - f0) * pow(1.0 - abs(dot(-r.dir, N)), 5.0);
            
            vec3 reflectDir = reflect(r.dir, N);
            vec3 envReflection = getEnvironment(reflectDir); 
            finalColor += throughput * envReflection * fresnelTerm;
            
            if (length(refracted) == 0.0) {
                r.dir = reflectDir;
                r.origin = hitPoint + N * EPSILON;
            } else {
                throughput *= (1.0 - fresnelTerm); 

    float dist = rec.t; // distance from the LAST hit to THIS hit

    // Absorption Coefficient (color of gem), vector for RGB
    vec3 absorbance = vec3(0.02, 0.02, 0.02); 

    // Beer-Lambert Law
    if (!rec.frontFace) {
        vec3 transmission = exp(-absorbance * dist);
        throughput *= transmission;
    }
                r.dir = refracted;
                r.origin = hitPoint - N * EPSILON;
            }
        }
    }
    return finalColor;
}

void main() {
    vec2 uv = (gl_FragCoord.xy / resolution) * 2.0 - 1.0;
    float imageAspectRatio = resolution.x / resolution.y; 
    float tanFov = tan(fov / 2.0);
    vec3 rayDir = normalize(cameraDir + (uv.x * imageAspectRatio * tanFov) * cameraRight + (uv.y * tanFov) * cameraUp);
    
    Ray r; r.origin = cameraPos; r.dir = rayDir;

    vec3 colR = traceWhitted(r, 1.70);
    vec3 colG = traceWhitted(r, 1.80);
    vec3 colB = traceWhitted(r, 1.90);
    
    vec3 finalColor = vec3(colR.r, colG.g, colB.b);
    finalColor *= 1.2;
    finalColor = pow(finalColor, vec3(1.2)); 
    FragColor = vec4(finalColor, 1.0);
}
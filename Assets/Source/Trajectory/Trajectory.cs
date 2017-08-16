using UnityEngine;
using System.Collections.Generic;

namespace Assets.Source.Trajectory
{
    public class Trajectory: MonoBehaviour
    {
        public float hitDistance = 20;

        public List<Transform> trajectoryPointsObjects = new List<Transform>();
        public LineRenderer firstTrajectoryLine, secondTrajectoryLine;
        public List<Vector3> trajectoryPointsPositions = new List<Vector3>();

        public Transform throwTransform;
        public Transform hitLocation, throwLocation;


        [Header("Trajectory 1")]
        public ushort trajectoryPointsCount;

        public float Vinitial;
        public float Vvertical;
        public float HeightInitial;
        public float Vhorizontal;
        public float Theta;
        public float Gravity;
        public float Range;
        public float Time;
        public float TimeDelta;

        [Header("Projectile")]
        public float forceBias = 0.0025f;
        public GameObject projectilePrefab;

        [Header("Collisions")]
        public bool detectedObstacle = false;
        public ushort trajectoryHitPointPositionIndex;
        public float velocityAtCollisionPoint;
        public float timeAtPoint;

        public Color reboundVecColor = new Color(0.866f, 53.7f, 1.0f, 1.0f);
        public Color hitNormalVecColor = new Color(1f, 0.54f, 0f, 1f);

        private Vector3 throwDirection, groundDirection, crossTgVector, reboundDirection;
        private Vector3 rayhitPosition, correspondingPointPos;
        private ushort rayCastCounter = 30, rayCastCounterMax = 30;
        private ushort lastIndex;


        private ushort delayCounter30 = 30;
        private ushort delayCounter10 = 10;
        private ushort delayCounter5 = 5;


        public void Start()
        {
            trajectoryPointsObjects = createPointsObjects(trajectoryPointsCount);
            trajectoryPointsPositions = createPointPosition(trajectoryPointsCount);
            firstTrajectoryLine.SetVertexCount(trajectoryPointsCount);
        }

        public void Update()
        {
            delayedUpdate5();

            //Vectors
            groundDirection = getGroundDirection();
            throwDirection = getThrowDirection();
            throwLocation.position = getThrowLocation();
            crossTgVector = getCrossTg(throwDirection, groundDirection);

            //Values
            Theta = getTheta(throwDirection, groundDirection);
            Vvertical = getVvertical(Vinitial, Theta);
            Vhorizontal = getVhorizontal(Vinitial, Theta);
            Time = getTime(Vvertical, Gravity);
            Time = getDeltaTime(Vvertical, Gravity, HeightInitial);
            TimeDelta = getDeltaTime(Vvertical, Gravity, HeightInitial);
            HeightInitial = getInitialHeight(transform);
            Range = getRange(Vinitial, Vvertical, Vhorizontal, Gravity, HeightInitial, Theta);
            hitLocation.position = getLandingPoint(throwLocation.position, groundDirection, HeightInitial, Range);
            correspondingPointPos = getCorrespondingToHitPointOnTrajectory(getRayHitPosition());
            velocityAtCollisionPoint = getVelocityAtDistance(getPointDistance(Range, trajectoryHitPointPositionIndex, lastIndex), Vvertical, Vinitial, Gravity, Theta);
            timeAtPoint = getTimeAtPoint(24, trajectoryPointsCount, Time);

            updateTrajectory();
            updateVelocityInitial();

            launchProjectile(throwDirection, Vinitial);

            


            
            //drawRay(transform.position, crossTgVector * 5, Color.magenta);
            //drawRay(transform.position, throwDirection * hitDistance, Color.red);
            //drawRay(transform.position, getGroundDirection() * hitDistance, Color.white);
            //drawWorldVectors();
        }

        private void delayedUpdate30()
        {
            delayCounter30--;
            if(delayCounter30 <= 0)
            {
                //code

                delayCounter30 = 30;
            }
        }

        private void delayedUpdate5()
        {
            delayCounter5--;
            if (delayCounter5 <= 0)
            {
                castRaysFromPoints();

                delayCounter5 = 5;
            }
        }





        private void updateTrajectory()
        {
            int firstIndex = 0, lastIndex = trajectoryPointsPositions.Count - 1;;
            Vector3 lastPointPos;
            float pointHeight, pointDistance, pointDistanceIfHit;

            //Locations
            for (int i = 0; i < trajectoryPointsPositions.Count; i++)
			{
                if(i != 0)
                {
                    lastPointPos = hitLocation.position;

                    pointDistanceIfHit = getPointDistance(Range / 2, i, lastIndex);
                    
                    pointDistance = getPointDistance(Range, i, lastIndex);
                    pointHeight = getPointHeight(Vvertical, Vhorizontal, pointDistance, Gravity, 0);

                    

                    trajectoryPointsPositions[i] = getPointPosition(lastPointPos, i, lastIndex, pointHeight);
                }

                if (i == firstIndex)
                    trajectoryPointsPositions[i] = throwLocation.position;
			}

            //updatePointsObjects(trajectoryPointsPositions);
            updateLineVertices(trajectoryPointsPositions);
        }

        private void updatePointsObjects(List<Vector3> pointsPositions)
        {
            Vector3 lookDirection;
            Quaternion newRotation;

            for (int i = 0; i < trajectoryPointsObjects.Count; i++)
            {
                trajectoryPointsObjects[i].position = pointsPositions[i];

                if(i != trajectoryPointsObjects.Count - 1)
                {
                    lookDirection = trajectoryPointsObjects[i+1].position - trajectoryPointsObjects[i].position;
                    newRotation = Quaternion.LookRotation(lookDirection, Vector3.up);
                    trajectoryPointsObjects[i].rotation = newRotation;
                }
            }
        }

        private void updateLineVertices(List<Vector3> pointsPositions)
        {
            int totalVertsCount = trajectoryPointsObjects.Count;
            firstTrajectoryLine.SetVertexCount(totalVertsCount);

            if (detectedObstacle)
            {
                totalVertsCount = getTrajectoryHitPointPosIndex() + 1;
                firstTrajectoryLine.SetVertexCount(totalVertsCount);
            }

            for (int i = 0; i < totalVertsCount; i++)
            {
                firstTrajectoryLine.SetPosition(i, pointsPositions[i]);
            }
        }

        private void updateVelocityInitial()
        {
            Vinitial += Input.GetAxis("Mouse ScrollWheel");
        }






        private float getPointDistance(float _range, int index, int lastIndex)
        {
            return (_range / lastIndex) * index;
        }

        private Vector3 getPointPosition(Vector3 lastPointPos, int index, int lastIndex, float heightIncrement)
        {
            float fractX, fractY, fractZ;
            Vector3 pos, lastPointDir;
            
            lastPointDir = lastPointPos - transform.position;

            fractX = (lastPointDir.x / lastIndex) * index;
            fractY = (lastPointDir.y / lastIndex) * index + heightIncrement;
            fractZ = (lastPointDir.z / lastIndex) * index;

            pos = new Vector3(fractX, fractY, fractZ) + transform.position;
            return pos;
        }

        private float getPointHeight(float Vvertical, float Vhorizontal, float pointDistance, float gravity, int x)
        {
            float pointHeight;
            float term1 = (Vvertical * pointDistance) / Vhorizontal;
            float term2 = 0.5f * -gravity * Mathf.Pow((pointDistance / Vhorizontal), 2);
            pointHeight = term1 + term2;

            return pointHeight;
        }

        private float getVelocityAtDistance(float pointDistance, float Vvertical, float Vinitial, float gravity, float theta)
        {
            float vAtDistance, vVerticalAtDistance;

            float term1 = Mathf.Pow(Vinitial, 2);
            float term2 = 2 * gravity * pointDistance * Mathf.Tan(Mathf.Deg2Rad * theta);
            float term3 = Mathf.Pow( ((gravity * pointDistance) / Vvertical) , 2);

            vAtDistance = Mathf.Sqrt(term1 - term2 + term3);
            vVerticalAtDistance = vAtDistance * Mathf.Sin(Mathf.Deg2Rad * theta);
            return vVerticalAtDistance;
        }

        private float getTimeAtPoint(ushort pointIndex, ushort totalPoints, float time)
        {
            return time / totalPoints * pointIndex;  
        }

        //private float getVelocityAtPoint(float timeAtPoint, float gravity, float time)
        //{
        //    //float middlePointVelocity = 0;
        //    //float targetPointTimeVal = timeAtPoint;
        //    //float halfWayTime = time / 2;
        //    //float deltaTime = timeAtPoint - 
        //}



        private void setReboundDirection(Vector3 dir)
        {
            reboundDirection = dir;
        }

        private Vector3 getReboundDirection()
        {
            return reboundDirection;
        }






        private Vector3 getRayHitPosition()
        {
            return rayhitPosition;
        }

        private void setRayHitPosition(Vector3 pos)
        {
            rayhitPosition = pos;
        }

        private void castRaysFromPoints()
        {
            Ray ray;
            Vector3 rayStart, rayDir, reboundDir, hitSurfaceNormalDir;
            RaycastHit hit;
            float distance;

            for (ushort i = 0; i < trajectoryPointsPositions.Count; i++)
            {
                if (i != trajectoryPointsPositions.Count - 1)
                {
                    rayStart = trajectoryPointsPositions[i];
                    rayDir = trajectoryPointsPositions[i + 1] - trajectoryPointsPositions[i];
                    distance = Vector3.Distance(trajectoryPointsPositions[i], trajectoryPointsPositions[i + 1]);
                    ray = new Ray(rayStart, rayDir);

                    if (Physics.Raycast(ray, out hit, distance))
                    {
                        if (hit.collider)
                        {
                            detectedObstacle = true;
                            hitSurfaceNormalDir = hit.normal;
                            reboundDir = 2 * Vector3.Dot(hitSurfaceNormalDir, -rayDir) * hitSurfaceNormalDir + rayDir;
                            setReboundDirection(reboundDir);
                            setTrajectoryHitPointPosIndex(i);
                            setRayHitPosition(trajectoryPointsPositions[i]);
                            Debug.DrawRay(ray.origin, ray.direction * distance, Color.red);
                            Debug.DrawRay(hit.point, hit.normal, hitNormalVecColor);
                            Debug.DrawRay(hit.point, reboundDir, reboundVecColor);
                            return;
                        }
                    }
                    Debug.DrawRay(ray.origin, ray.direction * distance, Color.green);
                }
            }
            detectedObstacle = false;
            setRayHitPosition(Vector3.zero);
        }

        private Vector3 getCorrespondingToHitPointOnTrajectory(Vector3 rayHitPos)
        {
            float biasX = 0.1f;
            float biasZ = 0.1f;

            if(rayHitPos != Vector3.zero)
            {
                for (int i = 0; i < trajectoryPointsPositions.Count; i++)
                {
                    if(rayHitPos.x - trajectoryPointsPositions[i].x <= biasX)
                    {
                        if(rayHitPos.z - trajectoryPointsPositions[i].z <= biasZ)
                        {
                            return trajectoryPointsPositions[i];
                        }
                    }
                }
            }
            return Vector3.zero;
        }

        private void setTrajectoryHitPointPosIndex(ushort i)
        {
            trajectoryHitPointPositionIndex = i;
        }

        private int getTrajectoryHitPointPosIndex()
        {
            return trajectoryHitPointPositionIndex;
        }






        private void snapObjectToPoint(Transform obj, Vector3 point)
        {
            obj.transform.position = point;
        }

        private Vector3 getCrossTg(Vector3 throwDir, Vector3 groundDir)
        {
            Vector3 temp = Vector3.Cross(groundDirection.normalized, throwDirection.normalized);
            return transform.InverseTransformVector(temp.normalized);
        }

        private Vector3 getGroundDirection()
        {
            return (Vector3)new Vector4(transform.root.forward.x, 0, transform.root.forward.z, 0).normalized;
        }

        private Vector3 getLandingPoint(Vector3 origin, Vector3 groundDirection, float initialHeight, float range)
        {
            Vector3 landingPoint = origin + (groundDirection.normalized * range);
            return landingPoint;
        }

        private Vector3 getThrowDirection()
        {
            return transform.forward;
        }
        
        private Vector3 getThrowLocation()
        {
            return transform.position;
        }


        



        private float getVvertical(float V_initial, float _theta)
        {
            float x = Mathf.Deg2Rad * _theta;
            return V_initial * Mathf.Sin(x);
        }

        private float getVhorizontal(float V_initial, float _theta)
        {
            float x = Mathf.Deg2Rad * _theta;
            return V_initial * Mathf.Cos(x);
        }

        private float getTheta(Vector3 throwDir, Vector3 groundDir)
        {
            if (crossTgVector.x < 0)
                return Vector3.Angle(throwDir.normalized, groundDir.normalized);
            else
                return -Vector3.Angle(throwDir.normalized, groundDir.normalized);
        }

        private float getTime(float Vvertical, float gravity)
        {
            return ( (0 - Vvertical) / -gravity) * 2;
        }

        private float getDeltaTime(float Vvertical, float gravity, float initialHeight)
        {
            float deltaPlus, deltaMin;
            float term1 = (-Vvertical - (Mathf.Sqrt(Mathf.Pow(Vvertical, 2) - 4 * (-4.9f) * initialHeight)));
            float term2 = 2 * -4.9f;
            deltaPlus = term1 / term2;

            return deltaPlus;
        }

        private float getRange(float Vhorizontal, float time)
        {
            return Vhorizontal * time;
        }

        private float getRange(float Vinitial, float Vvertical, float Vhorizontal, float gravity, float initialHeight, float _theta)
        {
            float term1 = Vhorizontal / gravity;
            float sineSquareTheta = Mathf.Pow( Mathf.Sin(Mathf.Deg2Rad * _theta) , 2);
            float vInitialSquare = Mathf.Pow(Vinitial, 2);
            float rootPart = Mathf.Sqrt(vInitialSquare * sineSquareTheta + 2 * gravity * initialHeight);
            float term2 = (Vvertical + rootPart);
            return term1 * term2;
        }

        private float getInitialHeight(Transform localTransform)
        {
            return localTransform.position.y;
        }
        





        private List<Transform> createPointsObjects(int num)
        {
            int x= 0;
            GameObject pointsParent = new GameObject("pointsParent");
            pointsParent.transform.parent = transform;

            List<Transform> pointsList = new List<Transform>(num);

            for (int i = 0; i < num; i++)
            {
                pointsList.Add(GameObject.CreatePrimitive(PrimitiveType.Sphere).transform);
            }

            foreach (Transform item in pointsList)
            {
                item.transform.localScale = new Vector3(.1f, .1f, .1f);
                item.transform.parent = pointsParent.transform;
                item.name = "point_"+x;
                item.gameObject.layer = 1 << 1;
                item.GetComponent<MeshRenderer>().material = (Material)Resources.Load("Materials/HitPoint_mat");
                if (item.gameObject.GetComponent<Collider>())
                    Destroy(item.GetComponent<Collider>());
                x++;
            }

            return pointsList;
        }

        private List<Vector3> createPointPosition(int num)
        {
            List<Vector3> positionsList = new List<Vector3>(num);

            for (int i = 0; i < num; i++)
            {
                positionsList.Add(Vector3.zero);
            }
            return positionsList;
        }






        private void launchProjectile(Vector3 launchDirection, float force)
        {
            float forceCompensationValue;
            Rigidbody rBody;
            Vector3 p_pos = transform.position;
            Quaternion p_rot = Quaternion.identity;

            //p_pos = new Vector3(p_pos.x, p_pos.y + 0.01f, p_pos.z);

            if(Input.GetMouseButtonDown(0))
            {
                GameObject p = (GameObject)GameObject.Instantiate(projectilePrefab, p_pos, p_rot);

                p.name = "projectile";
                rBody = p.GetComponent<Rigidbody>();
                forceCompensationValue = rBody.mass + forceBias;
                rBody.AddForce(launchDirection * force * forceCompensationValue, ForceMode.Impulse);

                Destroy(p, 5);
            }
        }






        private void drawWorldVectors()
        {
            Vector3 globalOrigin = Vector3.zero;

            //Player
            Vector3 cameraDir = transform.position;
            Debug.DrawRay(globalOrigin, cameraDir, Color.cyan);

            //Point Objects
            foreach (Vector3 v in trajectoryPointsPositions)
            {
                Debug.DrawRay(globalOrigin, v, Color.cyan);
            }
        }

        private void drawRay(Vector3 start, Vector3 dir, Color color)
        {
            Debug.DrawRay(start, dir, color);
        }
    }
}


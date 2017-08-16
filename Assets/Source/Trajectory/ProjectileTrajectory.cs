using System;
using System.Collections.Generic;
using UnityEngine;

namespace Assets.Source.Trajectory
{
    [Serializable]
    public struct TrajectoryInitialValues
    {
        public float velocityInitial;
        public float gravity;
    }
    [Serializable]
    public struct TrajectoryVariables
    {
        public float theta;
        public float velocityVertical;
        public float velocityHorizontal;
        public float timeDelta;
        public float range;
        public float initialHeight;
    }
    [Serializable]
    public struct TrajectoryProjectile
    {
        public GameObject projectilePrefab;
    }
    [Serializable]
    public struct TrajectoryPoints
    {
        public byte pointsCount;
        public Transform pointsParent;
        public List<Vector3> trajectoryPoints;
    }

    public class ProjectileTrajectory: MonoBehaviour
    {
        public LineRenderer trajectoryLine;

        [SerializeField] public TrajectoryInitialValues initialValues;
        [SerializeField] public TrajectoryVariables variables;
        [SerializeField] public TrajectoryProjectile projectile;
        [SerializeField] public TrajectoryPoints points;

        private TrajectoryEngine engine;
        private TrajectoryLine line;
        private TrajectoryRays rays;

        private bool _isManual = true;
        public bool IsManual
        {
            get { return _isManual; }
            set { _isManual = value; }
        }

        private float _autoInitialVelocity = 1;
        public float AutoInitialVelocity
        {
            get { return _autoInitialVelocity; }
            set { _autoInitialVelocity = value; }
        }

        private float _autoTheta = 1;
        public float AutoTheta
        {
            get { return _autoTheta; }
            set { _autoTheta = value; }
        }

        private bool isTrajectoryIntersecting = false;
        public bool IsTrajectoryIntersecting
        {
            get { return isTrajectoryIntersecting; }
            private set { isTrajectoryIntersecting = value; }
        }

        private byte _hitIndex = 0;
        public byte HitIndex
        {
            get { return _hitIndex; }
            set { _hitIndex = value; }
        }


        public void Start()
        {
            initializeTrajectory();
        }

        public void Update()
        {
            updateTrajectory(transform, trajectoryLine, IsManual);
        }

        public void setTrajectoryControl(bool isManual)
        {
            IsManual = isManual;
        }
        private void initializeTrajectory()
        {
            engine = new TrajectoryEngine();
            line = new TrajectoryLine(trajectoryLine, points.pointsCount);

            createTrajectoryPoints(points.pointsCount);

            rays = new TrajectoryRays(points.trajectoryPoints);
        }
        private void createTrajectoryPoints(byte count)
        {
            byte i = 0;

            for (i = 0; i < count; i++)
            {
                points.trajectoryPoints.Add(new Vector3(0, 0, 0));
            }
        }

        private void updateTrajectory(Transform tForm, LineRenderer trajectoryLine, bool isManual)
        {
            if (isManual)
                engine.updateEngine(tForm, initialValues.velocityInitial, initialValues.gravity, 12.34f, isManual);
            else
                engine.updateEngine(tForm, AutoInitialVelocity, initialValues.gravity, AutoTheta, !isManual);


            //display values
            variables.theta = engine.Theta;
            variables.velocityVertical = engine.VerticalVelocity;
            variables.velocityHorizontal = engine.HorizontalVelocity;
            variables.initialHeight = engine.InitialHeight;
            variables.timeDelta = engine.TimeDelta;
            variables.range = engine.Range;


            //points and line vertices
            updateTrajectoryPoints(tForm, this.trajectoryLine, initialValues.gravity);

            //rays
            rays.updateTrajectoryRays(points.trajectoryPoints);
            IsTrajectoryIntersecting = rays.WasObstacleDetected;
            HitIndex = rays.HitPointIndex;
        }
        private void updateTrajectoryPoints(Transform tForm, LineRenderer trajectoryLine, float gravity)
        {
            if (points.trajectoryPoints.Count == 0)
                Debug.LogError("Trajectory points list is null.");

            byte count = (byte)points.trajectoryPoints.Count;
            byte i = 0, lastIndex = (byte)(count - 1);

            for (i = 0; i < count; i++)
            {
                points.trajectoryPoints[i] = engine.updatePointPosition(tForm, i, lastIndex, gravity);
            }

            line.updateLineVertices(this.trajectoryLine, HitIndex, count, IsTrajectoryIntersecting, points.trajectoryPoints);
        }
    }

    public class TrajectoryEngine
    {
        private Vector3 launchDirection;
        private Vector3 groundDirection;
        private Vector3 crossLaunchGround;
        private Vector3 lastPointPosition;

        private float _theta;
        public float Theta
        {
            get
            {
                return _theta;
            }
            private set
            {
                if(_theta != value)
                    _theta = value;
            }
        }

        private float _verticalVelocity;
        public float VerticalVelocity
        {
          get { return _verticalVelocity; }
          private set 
          {
              if(_verticalVelocity != value)
                _verticalVelocity = value; 
          }
        }

        private float _horizontalVelocity;
        public float HorizontalVelocity
        {
            get { return _horizontalVelocity; }
            private set
            {
                if (_horizontalVelocity != value)
                    _horizontalVelocity = value;
            }
        }

        private float _initialHeight;
        public float InitialHeight
        {
            get
            {
                return _initialHeight;
            }
            private set
            {
                if (_initialHeight != value)
                    _initialHeight = value;
            }
        }

        private float _range;
        public float Range
        {
            get
            {
                return _range;
            }
            private set
            {
                if (_range != value)
                    _range = value;
            }
        }

        private float _timeDelta;
        public float TimeDelta
        {
            get
            {
                return _timeDelta;
            }
            private set
            {
                if (_timeDelta != value)
                    _timeDelta = value;
            }
        }


        public TrajectoryEngine()
        {

        }

        public void updateEngine(Transform trTransform, float velocityInitial, float gravity, float thetaB, bool isManual)
        {
            launchDirection = getThrowDirection(trTransform);
            groundDirection = getGroundDirection(trTransform);
            crossLaunchGround = getCrossVector(launchDirection, groundDirection, trTransform);
            lastPointPosition = getLastPointPosition(trTransform, groundDirection, Range);

            Theta = isManual ? updateTheta(crossLaunchGround, launchDirection, groundDirection) : thetaB; 
            VerticalVelocity = updateVelocityVertical(velocityInitial, Theta);
            HorizontalVelocity = updateVelocityHorizontal(velocityInitial, Theta);
            InitialHeight = updateInitialHeight(trTransform);
            TimeDelta = updateTimeDelta(VerticalVelocity, gravity, InitialHeight);
            Range = updateRange(HorizontalVelocity, TimeDelta);
        }

        public float updateVelocityVertical(float velocityInitial, float theta)
        {
            return velocityInitial * getSine(theta);
        }
        public float updateVelocityHorizontal(float velocityInitial, float theta)
        {
            return velocityInitial * getCosine(theta);
        }
        public float updateTheta(Vector3 crossVector, Vector3 a, Vector3 b)
        {
            if (crossVector.x < 0)
                return Vector3.Angle(a.normalized, b.normalized);
            else
                return -Vector3.Angle(a.normalized, b.normalized);
        }
        private float updateInitialHeight(Transform localTransform)
        {
            return localTransform.position.y;
        }
        private float updateRange(float horizontalVelocity, float timeDelta)
        {
            return horizontalVelocity * timeDelta;
        }
        private float updateTimeDelta(float verticalVelocity, float gravity, float initialHeight)
        {
            float halfGravity = gravity / 2;
            float term1 = (-verticalVelocity - (Mathf.Sqrt(Mathf.Pow(verticalVelocity, 2) - 4 * (-halfGravity) * initialHeight)));
            float term2 = 2 * -halfGravity;
            return term1 / term2;
        }


        public Vector3 updatePointPosition(Transform tForm, byte index, byte lastIndex, float gravity)
        {
            float fractX, fractY, fractZ;
            float pointDistance = updatePointDistance(Range, index, lastIndex);
            float pointHeight = updatePointHeight(VerticalVelocity, HorizontalVelocity, pointDistance, gravity);
            Vector3 lastPointDirection;

            lastPointDirection = lastPointPosition - tForm.position;

            fractX = (lastPointDirection.x / lastIndex) * index;
            fractY = (lastPointDirection.y / lastIndex) * index + pointHeight;
            fractZ = (lastPointDirection.z / lastIndex) * index;

            return new Vector3(fractX, fractY, fractZ) + tForm.position;
        }
        private float updatePointDistance(float range, byte index, byte lastIndex)
        {
            return (range / lastIndex) * index;
        }
        private float updatePointHeight(float verticalVelocity, float horizontalVelocity, float pointDistance, float gravity)
        {
            float term1 = (verticalVelocity * pointDistance) / horizontalVelocity;
            float term2 = 0.5f * -gravity * Mathf.Pow((pointDistance / horizontalVelocity), 2);
            return  term1 + term2;
        }

        public Vector3 getLastPointPosition(Transform tForm, Vector3 direction, float range)
        {
            if (range == null) return Vector3.zero;
            return tForm.position + (direction.normalized * range);
        }
        public Vector3 getThrowDirection(Transform tForm)
        {
            Debug.DrawRay(tForm.position, tForm.forward);
            return tForm.forward;
        }
        public Vector3 getGroundDirection(Transform tForm)
        {
            Debug.DrawRay(tForm.position, (Vector3)new Vector4(tForm.forward.x, 0, tForm.forward.z, 0));
            return (Vector3)new Vector4(tForm.forward.x, 0, tForm.forward.z, 0).normalized;
        }
        private Vector3 getCrossVector(Vector3 a, Vector3 b, Transform tForm)
        {
            Vector3 result = Vector3.Cross(b.normalized, a.normalized);
            Debug.DrawRay(tForm.position, tForm.InverseTransformVector(result.normalized));
            return tForm.InverseTransformVector(result.normalized);
        }
        private float getSine(float angle)
        {
            return Mathf.Sin(Mathf.Deg2Rad * angle);
        }
        private float getCosine(float angle)
        {
            return Mathf.Cos(Mathf.Deg2Rad * angle);
        }
    }

    public class TrajectoryLine
    {
        public LineRenderer trajectoryLine;

        public TrajectoryLine(LineRenderer line, byte count)
        {
            trajectoryLine = line;
            initializeLineVertices(line, count);
        }

        public void updateLineVertices(LineRenderer line, byte hitIndex, byte pointsCount, bool isIntersecting, List<Vector3> trajectoryPoints)
        {
            byte totalCount = pointsCount, i = 0;
            line.SetVertexCount(totalCount);

            if (isIntersecting)
            {
                totalCount = (byte)(hitIndex + 1);
                line.SetVertexCount(totalCount);
            }

            for (i = 0; i < totalCount; i++)
            {
                trajectoryLine.SetPosition(i, trajectoryPoints[i]);
            }
        }
        private void initializeLineVertices(LineRenderer line, byte count)
        {
            line.SetVertexCount(count);
            line.SetWidth(0.4f, 0.2f);
        }
    }

    public class TrajectoryRays
    {
        public List<Vector3> trajectoryPoints;

        private byte _hitPointIndex = 0;
        public byte HitPointIndex
        {
            get { return _hitPointIndex; }
            private set { _hitPointIndex = value; }
        }

        private Vector3 _rayHitPosition = Vector3.zero;
        public Vector3 RayHitPosition
        {
            get { return _rayHitPosition; }
            private set { _rayHitPosition = value; }
        }

        private Vector3 _reflectedHitDirection = Vector3.zero;
        public Vector3 ReflectedHitDirection
        {
            get { return _reflectedHitDirection; }
            private set { _reflectedHitDirection = value.normalized; }
        }

        private bool _wasObstacleDetected = false;
        public bool WasObstacleDetected
        {
            get { return _wasObstacleDetected; }
            private set { _wasObstacleDetected = value; }
        }


        private byte _updateCounterMax = 20;
        private byte _updateCounterMin = 10;
        private byte _updateCounter = 0;
        private byte _lastPointIndex;
        private byte _pointsCount;

        public TrajectoryRays(List<Vector3> points)
        {
            initializeTrajectoryRays(points);
        }

        public void initializeTrajectoryRays(List<Vector3> points)
        {
            if (points.Count == 0)
                Debug.LogError("Trajectory points list is null.");
           
            trajectoryPoints = points;
            _updateCounter = _updateCounterMax;

            _pointsCount = (byte)points.Count;
            _lastPointIndex = (byte)(_pointsCount - 1);
        }
        public void updateTrajectoryRays(List<Vector3> points)
        {
            _updateCounter--;
            if (_updateCounter <= _updateCounterMin)
            {
                _updateCounter = _updateCounterMax;
                
                //update
                castRaysFromPoints(points, _pointsCount, _lastPointIndex);
            }
        }
        private void castRaysFromPoints(List<Vector3> points, byte count, byte lastPointIndex)
        {
            Ray ray;
            RaycastHit hit;
            Vector3 rayOrigin, rayDir;
            Vector3 reflectDir, hitNormalDir;
            float distance;

            byte i = 0;

            for (i = 0; i < count; i++)
            {
                if(i != lastPointIndex)
                {
                    rayOrigin = points[i];
                    rayDir = points[i + 1] - points[i];
                    distance = Vector3.Distance(points[i], points[i + 1]);
                    ray = new Ray(rayOrigin, rayDir);

                    if(Physics.Raycast(ray, out hit, distance))
                    {
                        if(hit.collider)
                        { 
                            WasObstacleDetected = true;

                            HitPointIndex = i;
                            hitNormalDir = hit.normal;
                            RayHitPosition = hit.point;
                            ReflectedHitDirection = getReflectDirection(hitNormalDir, rayDir);

                            Debug.DrawRay(ray.origin, ray.direction * distance, Color.red);
                            Debug.DrawRay(hit.point, ReflectedHitDirection, Color.yellow);
                            return;
                        }
                    }
                    Debug.DrawRay(ray.origin, ray.direction * distance, Color.green);
                }
            }
            WasObstacleDetected = false;
            RayHitPosition = Vector3.zero;
            ReflectedHitDirection = Vector3.zero;
            HitPointIndex = 0;
        }

        private Vector3 getReflectDirection(Vector3 a, Vector3 b)
        {
            return 2 * Vector3.Dot(a, -b) * a + b;
        }
    }
}

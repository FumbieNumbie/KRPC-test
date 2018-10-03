using Google.Protobuf;
using System;
using System.Collections.Generic;
using System.Runtime.Serialization;

#if NET35
using systemAlias = global::KRPC.Client.Compatibility;
using genericCollectionsAlias = global::KRPC.Client.Compatibility;
#else
using systemAlias = global::System;
using genericCollectionsAlias = global::System.Collections.Generic;
#endif

namespace KRPC.Client.Services.Trajectories
{
    /// <summary>
    /// Extension methods for Trajectories service.
    /// </summary>
    public static class ExtensionMethods
    {
        /// <summary>
        /// Create an instance of the Trajectories service.
        /// </summary>
        public static global::KRPC.Client.Services.Trajectories.Service Trajectories(this global::KRPC.Client.IConnection connection)
        {
            return new global::KRPC.Client.Services.Trajectories.Service(connection);
        }
    }

    /// <summary>
    /// Trajectories service.
    /// </summary>
    public class Service
    {
        global::KRPC.Client.IConnection connection;

        internal Service(global::KRPC.Client.IConnection serverConnection)
        {
            connection = serverConnection;
        }

        /// <summary>
        /// Returns the corrected direction or Null if no active vessel or set target.
        /// </summary>
        [global::KRPC.Client.Attributes.RPCAttribute("Trajectories", "CorrectedDirection")]
        public systemAlias::Tuple<double,double,double> CorrectedDirection()
        {
            ByteString _data = connection.Invoke("Trajectories", "CorrectedDirection");
            return (systemAlias::Tuple<double,double,double>)global::KRPC.Client.Encoder.Decode(_data, typeof(systemAlias::Tuple<double,double,double>), connection);
        }

        /// <summary>
        /// Returns trajectory patch EndTime or Null if no active vessel or calculated trajectory.
        /// See GetTimeTillImpact for remaining time until impact.
        /// </summary>
        [global::KRPC.Client.Attributes.RPCAttribute("Trajectories", "GetEndTime")]
        public double GetEndTime()
        {
            ByteString _data = connection.Invoke("Trajectories", "GetEndTime");
            return (double)global::KRPC.Client.Encoder.Decode(_data, typeof(double), connection);
        }

        /// <summary>
        /// Returns the calculated impact position of the trajectory in lat and long.
        /// </summary>
        [global::KRPC.Client.Attributes.RPCAttribute("Trajectories", "GetImpactLatLong")]
        public systemAlias::Tuple<double, double> GetImpactLatLong()
        {
            ByteString _data = connection.Invoke("Trajectories", "GetImpactLatLong");
            return (systemAlias::Tuple<double, double>)global::KRPC.Client.Encoder.Decode(_data, typeof(systemAlias::Tuple<double, double>), connection);
        }

        /// <summary>
        /// Returns the calculated impact position of the trajectory or Null if no active vessel or calculated trajectory.
        /// </summary>
        [global::KRPC.Client.Attributes.RPCAttribute("Trajectories", "GetImpactPosition")]
        public systemAlias::Tuple<double,double,double> GetImpactPosition()
        {
            ByteString _data = connection.Invoke("Trajectories", "GetImpactPosition");
            return (systemAlias::Tuple<double,double,double>)global::KRPC.Client.Encoder.Decode(_data, typeof(systemAlias::Tuple<double,double,double>), connection);
        }

        /// <summary>
        /// Returns the calculated impact velocity of the trajectory or Null if no active vessel or calculated trajectory.
        /// </summary>
        [global::KRPC.Client.Attributes.RPCAttribute("Trajectories", "GetImpactVelocity")]
        public systemAlias::Tuple<double,double,double> GetImpactVelocity()
        {
            ByteString _data = connection.Invoke("Trajectories", "GetImpactVelocity");
            return (systemAlias::Tuple<double,double,double>)global::KRPC.Client.Encoder.Decode(_data, typeof(systemAlias::Tuple<double,double,double>), connection);
        }

        /// <summary>
        /// Returns the remaining time until Impact in seconds or Null if no active vessel or calculated trajectory.
        /// </summary>
        [global::KRPC.Client.Attributes.RPCAttribute("Trajectories", "GetTimeTillImpact")]
        public double GetTimeTillImpact()
        {
            ByteString _data = connection.Invoke("Trajectories", "GetTimeTillImpact");
            return (double)global::KRPC.Client.Encoder.Decode(_data, typeof(double), connection);
        }

        /// <summary>
        /// Returns true if a target has been set, false if not, or Null if no active vessel.
        /// </summary>
        [global::KRPC.Client.Attributes.RPCAttribute("Trajectories", "HasTarget")]
        public bool HasTarget()
        {
            ByteString _data = connection.Invoke("Trajectories", "HasTarget");
            return (bool)global::KRPC.Client.Encoder.Decode(_data, typeof(bool), connection);
        }

        /// <summary>
        /// Returns the planned direction or Null if no active vessel or set target.
        /// </summary>
        [global::KRPC.Client.Attributes.RPCAttribute("Trajectories", "PlannedDirection")]
        public systemAlias::Tuple<double,double,double> PlannedDirection()
        {
            ByteString _data = connection.Invoke("Trajectories", "PlannedDirection");
            return (systemAlias::Tuple<double,double,double>)global::KRPC.Client.Encoder.Decode(_data, typeof(systemAlias::Tuple<double,double,double>), connection);
        }

        /// <summary>
        /// Set the trajectories target to a latitude, longitude and altitude at the HomeWorld.
        /// </summary>
        [global::KRPC.Client.Attributes.RPCAttribute("Trajectories", "SetTarget")]
        public void SetTarget(double lat, double lon, double alt = 2.0)
        {
            var _args = new ByteString[] {
                global::KRPC.Client.Encoder.Encode (lat, typeof(double)),
                global::KRPC.Client.Encoder.Encode (lon, typeof(double)),
                global::KRPC.Client.Encoder.Encode (alt, typeof(double))
            };
            connection.Invoke("Trajectories", "SetTarget", _args);
        }

        /// <summary>
        /// Modifies the AlwaysUpdate value in the settings page.
        /// </summary>
        [global::KRPC.Client.Attributes.RPCAttribute("Trajectories", "get_AlwaysUpdate")]
        public bool AlwaysUpdate
        {
            get
            {
                ByteString _data = connection.Invoke("Trajectories", "get_AlwaysUpdate");
                return (bool)global::KRPC.Client.Encoder.Decode(_data, typeof(bool), connection);
            }
            set
            {
                var _args = new ByteString[] {
                    global::KRPC.Client.Encoder.Encode (value, typeof(bool))
                };
                connection.Invoke("Trajectories", "set_AlwaysUpdate", _args);
            }
        }

        /// <summary>
        /// Returns the version number of trajectories in a string formated as Major.Minor.Patch i.e. 2.1.0
        /// </summary>
        [global::KRPC.Client.Attributes.RPCAttribute("Trajectories", "get_GetVersion")]
        public string GetVersion
        {
            get
            {
                ByteString _data = connection.Invoke("Trajectories", "get_GetVersion");
                return (string)global::KRPC.Client.Encoder.Decode(_data, typeof(string), connection);
            }
        }

        /// <summary>
        /// Returns the major version number of trajectories
        /// </summary>
        [global::KRPC.Client.Attributes.RPCAttribute("Trajectories", "get_GetVersionMajor")]
        public int GetVersionMajor
        {
            get
            {
                ByteString _data = connection.Invoke("Trajectories", "get_GetVersionMajor");
                return (int)global::KRPC.Client.Encoder.Decode(_data, typeof(int), connection);
            }
        }

        /// <summary>
        /// Returns the minor version number of trajectories
        /// </summary>
        [global::KRPC.Client.Attributes.RPCAttribute("Trajectories", "get_GetVersionMinor")]
        public int GetVersionMinor
        {
            get
            {
                ByteString _data = connection.Invoke("Trajectories", "get_GetVersionMinor");
                return (int)global::KRPC.Client.Encoder.Decode(_data, typeof(int), connection);
            }
        }

        /// <summary>
        /// Returns the patch version number of trajectories
        /// </summary>
        [global::KRPC.Client.Attributes.RPCAttribute("Trajectories", "get_GetVersionPatch")]
        public int GetVersionPatch
        {
            get
            {
                ByteString _data = connection.Invoke("Trajectories", "get_GetVersionPatch");
                return (int)global::KRPC.Client.Encoder.Decode(_data, typeof(int), connection);
            }
        }

        /// <summary>
        /// Set the trajectories descent profile to Prograde.
        /// </summary>
        [global::KRPC.Client.Attributes.RPCAttribute("Trajectories", "get_ProgradeEntry")]
        public bool ProgradeEntry
        {
            get
            {
                ByteString _data = connection.Invoke("Trajectories", "get_ProgradeEntry");
                return (bool)global::KRPC.Client.Encoder.Decode(_data, typeof(bool), connection);
            }
            set
            {
                var _args = new ByteString[] {
                    global::KRPC.Client.Encoder.Encode (value, typeof(bool))
                };
                connection.Invoke("Trajectories", "set_ProgradeEntry", _args);
            }
        }

        /// <summary>
        /// Set the trajectories descent profile to Prograde.
        /// </summary>
        [global::KRPC.Client.Attributes.RPCAttribute("Trajectories", "get_RetrogradeEntry")]
        public bool RetrogradeEntry
        {
            get
            {
                ByteString _data = connection.Invoke("Trajectories", "get_RetrogradeEntry");
                return (bool)global::KRPC.Client.Encoder.Decode(_data, typeof(bool), connection);
            }
            set
            {
                var _args = new ByteString[] {
                    global::KRPC.Client.Encoder.Encode (value, typeof(bool))
                };
                connection.Invoke("Trajectories", "set_RetrogradeEntry", _args);
            }
        }
    }
}

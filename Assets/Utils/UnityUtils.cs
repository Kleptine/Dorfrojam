using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.SceneManagement;
using Debug = UnityEngine.Debug;
using Object = UnityEngine.Object;
#if UNITY_EDITOR
using UnityEditor.Build.Content;

#endif

namespace Utilities.Unity
{
    /// <summary>Contains a smattering of general utilties for Unity projects.</summary>
    public static class UnityUtils
    {
        /// <summary>
        ///     Scales a transform around a pivot, rather than its own origin. The scale factor is multiplied with the current
        ///     scale.
        /// </summary>
        /// <param name="target">Transform to scale.</param>
        /// <param name="pivotWorld">World space position of the pivot.</param>
        /// <param name="scale">Scale factor to apply.</param>
        public static void ScaleAround(this Transform target, Vector3 pivotWorld, Vector3 scale)
        {
            Vector3 diff = target.position - pivotWorld;
            diff.Scale(scale);
            target.position = pivotWorld + diff;

            target.localScale.Scale(scale);
        }

        /// <summary>Returns the children of this rect transform as an iterable.</summary>
        public static IEnumerable<RectTransform> GetChildren(this RectTransform root)
        {
            for (int i = 0; i < root.childCount; i++)
            {
                yield return root.GetChild(i) as RectTransform;
            }
        }

        /// <summary>Returns the children of this gameobject as an iterable.</summary>
        public static IEnumerable<Transform> GetChildren(this GameObject root)
        {
            return root.transform.GetChildren();
        }

        /// <summary>Returns the children of this transform as an iterable.</summary>
        public static IEnumerable<Transform> GetChildren(this Transform root)
        {
            for (int i = 0; i < root.childCount; i++)
            {
                yield return root.GetChild(i);
            }
        }

        /// <summary>Returns the first child with the given name.</summary>
        public static Transform FirstChildOrDefault(this Transform parent, string name)
        {
            if (parent.childCount == 0)
            {
                return null;
            }

            Transform result = null;
            for (int i = 0; i < parent.childCount; i++)
            {
                Transform child = parent.GetChild(i);
                if (child.name == name)
                {
                    return child;
                }

                result = FirstChildOrDefault(child, name);
            }

            return result;
        }

        /// <summary>Returns the first component of the given type in this object or any of its parents.</summary>
        /// <param name="component">The component to search from.</param>
        /// <param name="includeInactive">Whether to look at disabled objects when searching.</param>
        /// <returns>The first component that matches this type.</returns>
        /// <remarks>
        ///     Unity does not provide the 'includeInactive' overload for MonoBehaviour.GetComponentInParent, even though it
        ///     does provide it for GameObject.GetComponentInParent. This provides that missing function.
        /// </remarks>
        public static T GetComponentInParent<T>(this Component component, bool includeInactive) where T : Component
        {
            // todo(fixforship): This allocates. We should cache this list with a pool.
            // Might also be able to use GameObject.GetComponentInParent(bool), but I'm not sure that actually
            // has the same behaviour, when called on a component of type T.
            T[] list = component.GetComponentsInParent<T>(includeInactive);
            return list.Length > 0 ? list[0] : null;
        }

        /// <summary>Returns the set of active colliders on this gameobject and children.</summary>
        public static IEnumerable<Collider> GetActiveCollidersInChildren(this GameObject gameObject)
        {
            return gameObject.GetComponentsInChildren<Collider>().Where(c => c.enabled);
        }

        /// <summary>Returns the set of active colliders on this gameobject and children.</summary>
        public static IEnumerable<Collider> GetActiveCollidersInChildren(this Component component)
        {
            return component.GetComponentsInChildren<Collider>().Where(c => c.enabled);
        }

        /// <summary>Returns the set of active colliders on this gameobject.</summary>
        public static IEnumerable<Collider> GetActiveColliders(this GameObject gameObject)
        {
            return gameObject.GetComponents<Collider>().Where(c => c.enabled);
        }

        /// <summary>Returns the set of active colliders on this gameobject.</summary>
        public static IEnumerable<Collider> GetActiveColliders(this Component component)
        {
            return component.GetComponents<Collider>().Where(c => c.enabled);
        }

        /// <summary>Finds the first component in a child of this scene of type T.</summary>
        /// <param name="scene">Scene to search through.</param>
        /// <param name="includeInactive">Whether to search GameObjects if they are inactive.</param>
        /// <typeparam name="T">The type to search for.</typeparam>
        /// <returns>The first found component or null if none is found.</returns>
        public static T GetComponentInChildren<T>(this Scene scene, bool includeInactive = false) where T : class
        {
            if (!scene.IsValid())
            {
                return null;
            }

            GameObject[] roots = scene.GetRootGameObjects();
            foreach (GameObject root in roots)
            {
                if (!includeInactive && !root.activeSelf)
                {
                    continue;
                }

                var found = root.GetComponentInChildren<T>(includeInactive);
                if (found != null)
                {
                    return found;
                }
            }

            return null;
        }

        /// <summary>Finds all components in a child of this scene of type T.</summary>
        /// <param name="scene">Scene to search through.</param>
        /// <param name="includeInactive">Whether to search GameObjects if they are inactive.</param>
        /// <typeparam name="T">The type to search for. Can be an in</typeparam>
        /// <returns>The first found component or null if none is found.</returns>
        public static List<T> GetComponentsInChildren<T>(this Scene scene, bool includeInactive = false)
        {
            var results = new List<T>();
            if (!scene.IsValid())
            {
                return results;
            }

            GameObject[] roots = scene.GetRootGameObjects();
            foreach (GameObject root in roots)
            {
                if (!includeInactive && !root.activeSelf)
                {
                    continue;
                }

                T[] found = root.GetComponentsInChildren<T>(includeInactive);
                results.AddRange(found);
            }

            return results;
        }

        /// <summary>
        ///     Returns the best path we can find for this UnityEngine.Object. If it's a gameobject, or a component, we will
        ///     use the scene hierarchy path. If it's anything else, we just use the name and type.
        /// </summary>
        /// <returns>A string in the form "(Scene)/rootobject/someobject/otherobject/thisobject"</returns>
        public static string GetPathString(this Object obj)
        {
            if (obj == null)
            {
                return "null-reference";
            }

            switch (obj)
            {
                case Component comp:
                    if (comp.gameObject == null)
                    {
                        return $"null-gameobject:{comp.GetType().Name}";
                    }

                    return $"{comp.gameObject.GetPath()}:{comp.GetType().Name}";
                case GameObject gObj:
                    return gObj.gameObject.GetPath();
                default:
                    string objectName = obj.ToString();

                    // In the editor, check to see if this object is part of an asset, and return that string instead.
#if UNITY_EDITOR
                    var path = AssetDatabase.GetAssetPath(obj);
                    if (path != null)
                    {
#if UNITY_EDITOR
                        // Returns empty if the asset is a sub-asset on another.
                        if (string.IsNullOrEmpty(path))
                        {
                            ObjectIdentifier.TryGetObjectIdentifier(obj, out ObjectIdentifier ident);
                            path = ident.filePath;
                            return $"(Asset)/{path}/{obj.name}";
                        }
#endif

                        path = path.Replace("Assets/", "");
                        return $"(Asset)/{path}/{objectName}";
                    }
#endif

                    return objectName;
            }
        }

        /// <summary>Returns the full path to this transform including parents and the name of this gameobject.</summary>
        /// <returns>A string in the form "rootobject/someobject/otherobject/thisobject"</returns>
        public static string GetPath(this Transform transform)
        {
            if (transform == null)
            {
                return "???/(null-transform)";
            }

            if (transform.parent == null)
            {
                string scene = transform.gameObject.scene.name; // prefabs have null scenes
                if (scene == null)
                {
#if UNITY_EDITOR
                    string path = AssetDatabase.GetAssetPath(transform).Replace("Assets/", "");
                    return $"(Prefab)/{path}"; // Skip the root transform name on the prefab for conciseness.
#else
                    return $"(Prefab)/{transform.gameObject.name}";
#endif
                }

                return $"({scene})/{transform.name}";
            }

            return transform.parent.GetPath() + "/" + transform.name;
        }

        /// <summary>Returns the full path to this transform including parents and the name of this gameobject.</summary>
        /// <returns>A string in the form "rootobject/someobject/otherobject/thisobject"</returns>
        public static string GetPath(this GameObject gObj)
        {
            if (gObj == null)
            {
                return "null-reference";
            }

            return gObj.transform.GetPath();
        }

        /// <summary>Gets the names of the hierarchy objects above and including this object.</summary>
        public static void GetHierarchyPath(this Transform transform, List<string> outputPath)
        {
            var t = transform;
            while (t != null)
            {
                outputPath.Add(t.name);
                t = t.parent;
            }

            outputPath.Reverse();
        }

        /// <summary>
        ///     Prints out a summary of the number of each <see cref="System.Type" /> of <see cref="UnityEngine.Object" />
        ///     that currently exists in memory. Prints the count of each type on a log line.
        /// </summary>
        /// <remarks>Printing this much stuff is slow. Use for memory debugging.</remarks>
        public static void PrintUnityObjectsAllocated()
        {
            Object[] objcts = Object.FindObjectsOfType<Object>();
            Debug.Log("================================================");
            Debug.Log("================================================");
            Debug.Log("Total objects: " + objcts.Length);
            objcts.GroupBy(x => x.GetType().FullName).OrderByDescending(g => g.Count())
                  .Select(g => $"{g.Key}: {g.Count()}").PrintOnLines();
        }

        /// <summary>Prints out a set of items, one on each console line.</summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="source"></param>
        public static void PrintOnLines<T>(this IEnumerable<T> source, string prepend = null)
        {
            foreach (T e in source)
            {
                Debug.Log(prepend != null ? prepend + e : e.ToString());
            }
        }

        /// <summary>Whether this object is a part of a prefab asset. Ie. It is not in a scene.</summary>
        public static bool IsPrefabAsset(this GameObject obj)
        {
            // I'm not sure this is a sufficient check, during edit mode.
            Assert.IsTrue(Application.IsPlaying(obj));
            return obj.scene.name == null;
        }

        public enum PathType
        {
            AssetsRelative,
            ProjectRelative
        }

        /// <summary>
        ///     Returns the folder path to the associated data folder for the Scene. Ie. the folder next to the scene file
        ///     that is named the same as the scene.
        /// </summary>
        /// <param name="assetsRelative">Whether to include the Assets/ prefix.</param>
        public static string GetAssociatedDataFolder(this Scene scene, PathType pathType = PathType.ProjectRelative)
        {
            if (!scene.path.EndsWith(".unity"))
            {
                throw new Exception($"Scene path for [{scene.name}] is invalid. Scene path: [{scene.path}]");
            }

            // Remove .unity off the back.
            var p = scene.path.Remove(scene.path.Length - 6, 6);

            // Remove "Assets/" off the front.
            if (pathType == PathType.AssetsRelative)
            {
                p = p.Remove(0, 7);
            }

            return p;
        }

        /// <summary>Enables or disables collisions between all of the colliders on the following Rigidbodies.</summary>
        /// <remarks>This is O(n^2) with the number of colliders on each body. Be sensible.</remarks>
        /// <seealso cref="Physics.IgnoreCollision(UnityEngine.Collider,UnityEngine.Collider,bool)" />
        public static void EnableCollisions(Rigidbody body1, Rigidbody body2, bool enable)
        {
            foreach (Collider c in body1.GetComponentsInChildren<Collider>())
            {
                foreach (var c2 in body2.GetComponentsInChildren<Collider>())
                {
                    Physics.IgnoreCollision(c, c2, !enable);
                }
            }
        }

        /// <summary>Checks if a point is within the camera's view frustum.</summary>
        public static bool PointInFrustum(this Camera cam, Vector3 worldPosition)
        {
            var pos = cam.WorldToViewportPoint(worldPosition);
            return pos.x > 0 && pos.x < 1 && pos.y > 0 && pos.y < 1 && pos.z > 0;
        }

        /// <summary>Prints out the world transform with full precision.</summary>
        public static void PrintWorldTransform(Transform t)
        {
            Debug.Log(
                $"Pos: [{t.position.ToString("R")}] Rot: [{t.rotation.ToString("R")}] Scl: [{t.lossyScale.ToString("R")}]  Path: [{t.GetPathString()}]");
        }

#if UNITY_EDITOR
        /// <summary>Returns the Unity project relative path for this directory. In the form of Assets/Directory/Name</summary>
        public static string GetProjectRelative(this DirectoryInfo directory)
        {
            string assetsParentPath = Application.dataPath.Replace("/Assets", "") + "/";
            string dirPath = directory.FullName.Replace("\\", "/");
            return dirPath.Replace(assetsParentPath, "");
        }

        /// <summary>Helper to call <see cref="GlobalObjectId.TryParse" /> and asset it succeeds.</summary>
        public static GlobalObjectId ParseGlobalObjectId(string id)
        {
            bool success = GlobalObjectId.TryParse(id, out GlobalObjectId outId);
            if (!success)
            {
                Debug.LogError($"Couldn't parse GlobalObjectId [{id}]");
                return new GlobalObjectId();
            }

            return outId;
        }

        /// <summary>
        ///     Converts a Prefab GlobalObjectId (the id assigned to an object in the editor, while it's a prefab) to an
        ///     Unpacked GlobalObjectId (the id assigned to the same object after Unity 'unpacks' it during a build).
        /// </summary>
        /// <remarks>https://uninomicon.com/doku.php?id=globalobjectid</remarks>
        public static GlobalObjectId ConvertPrefabGidToUnpackedGid(GlobalObjectId id)
        {
            ulong fileId = (id.targetObjectId ^ id.targetPrefabId) & 0x7fffffffffffffff;
            bool success = GlobalObjectId.TryParse(
                $"GlobalObjectId_V1-{id.identifierType}-{id.assetGUID}-{fileId}-0",
                out GlobalObjectId unpackedGid);
            Assert.IsTrue(success);
            return unpackedGid;
        }

#endif

        /// <summary>A version of AssetDatabase.GetAssetPath() that can be used in non-editor code.</summary>
        /// <remarks>Useful for interspersing logging just for the editor.</remarks>
        public static string GetAssetPathSafe(this Object asset)
        {
#if UNITY_EDITOR

            return AssetDatabase.GetAssetPath(asset);
#else
            return "[editor-only value]";
#endif
        }

        /// <summary>Blocks the thread, running the given process and returns Output stream and Error stream.</summary>
        /// <param name="startInfo"></param>
        /// <param name="captureOutput">
        ///     If true, will capture all output and return these values from the function. If false, shows
        ///     the output in a popup terminal window.
        /// </param>
        /// <param name="suppressTimeoutError">
        ///     If true, will not log the error when a program times out, instead killing it
        ///     silently.
        /// </param>
        /// <param name="timeoutMs">How long to wait before killing the process.</param>
        /// <returns>STDOUT, STDERR, Finished Process Handle</returns>
        public static (string, string, Process finishedProcess) RunProcess(
            ProcessStartInfo startInfo, bool captureOutput = true,
            bool suppressTimeoutError = false, int timeoutMs = 5000)
        {
            if (captureOutput)
            {
                startInfo.UseShellExecute = false;
                startInfo.RedirectStandardOutput = true;
                startInfo.RedirectStandardError = true;
            }
            else
            {
                startInfo.UseShellExecute = true;
                startInfo.CreateNoWindow = false;
                startInfo.RedirectStandardOutput = false;
                startInfo.RedirectStandardError = false;
            }

            Process process = Process.Start(startInfo);
            if (process == null)
            {
                throw new Exception("No process was started.");
            }

            StringBuilder outputStringBuilder = new StringBuilder("");
            StringBuilder errorStringBuilder = new StringBuilder("");
            if (captureOutput)
            {
                process.OutputDataReceived += (sender, args) =>
                {
                    if (!string.IsNullOrEmpty(args.Data))
                    {
                        // Add the text to the collected output.
                        outputStringBuilder.Append(args.Data);
                        outputStringBuilder.Append(Environment.NewLine);
                    }
                };
                process.ErrorDataReceived += (sender, args) =>
                {
                    // Collect the sort command output.
                    if (!string.IsNullOrEmpty(args.Data))
                    {
                        // Add the text to the collected output.
                        errorStringBuilder.Append(args.Data);
                        errorStringBuilder.Append(Environment.NewLine);
                    }
                };

                process.BeginOutputReadLine();
                process.BeginErrorReadLine();
            }

            // throws on timeout exceeded
            if (!process.WaitForExit(timeoutMs))
            {
                if (!suppressTimeoutError)
                {
                    Debug.LogError(
                        $"Process [{process.ProcessName}] did not finish within timeout [{timeoutMs}ms]. Killing.");
                }

                process.Kill();
            }

            process.WaitForExit(500);

            if (!process.HasExited)
            {
                Debug.LogError(
                    $"Process [{process.ProcessName}] could not be killed [{timeoutMs}ms]. Killing.");
            }

            string outputString = outputStringBuilder.ToString();
            string errorString = errorStringBuilder.ToString();

            // empty if we didn't have capture output set.
            return (outputString, errorString, process);
        }
    }
}

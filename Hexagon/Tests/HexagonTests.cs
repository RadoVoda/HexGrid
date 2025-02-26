
using Hexagon;
using NUnit.Framework;
using System;
using System.Security.Cryptography;
using Unity.Collections;
using UnityEditor.ShaderKeywordFilter;
using UnityEngine.TestTools;

namespace Hexagon
{
    public class HexagonTests
    {
        static readonly Hex hexA = new Hex(0, 1);
        static readonly Hex hexB = new Hex(1, 1);
        static readonly Hex hexC = new Hex(11, 17);

        [Test]
        public void HexNeighbours()
        {
            Assert.IsTrue(hexA.IsNeighbour(Hex.Zero));
            Assert.IsTrue(hexA.IsNeighbour(hexB));

            var refStruct = hexA.Neighbours;
            var array = hexA.GetNeighbours();
            bool result = refStruct.Count == array.Length;

            if (result)
            {
                for (int i = 0; i < array.Length; i++)
                {
                    var temp = refStruct[i];

                    if (temp != array[i])
                    {
                        result = false;
                        break;
                    }
                }
            }

            Assert.IsTrue(result);
        }

        [Test]
        public void HexRing()
        {
            var refStruct = hexA.HexesAtRange(4);
            var array = hexA.GetHexesAtRange(4);
            bool result = refStruct.Count == array.Length;

            if (result)
            {
                for (int i = 0; i < array.Length; i++)
                {
                    var temp = refStruct[i];

                    if (temp != array[i])
                    {
                        result = false;
                        break;
                    }
                }
            }

            Assert.IsTrue(result);
        }

        [Test]
        public void HexArea()
        {
            var refStruct = hexA.HexesInRadius(4);
            var array = hexA.GetHexesInRadius(4);
            bool result = refStruct.Count == array.Length;

            if (result)
            {
                for (int i = 0; i < array.Length; i++)
                {
                    var temp = refStruct[i];

                    if (temp != array[i])
                    {
                        result = false;
                        break;
                    }
                }
            }

            Assert.IsTrue(result);
        }

        [Test]
        public void HexLine()
        {
            var refStruct = hexA.HexesInLine(hexC);
            var array = hexA.GetHexesInLine(hexC);
            bool result = refStruct.Count == array.Length;

            if (result)
            {
                for (int i = 0; i < array.Length; i++)
                {
                    var temp = refStruct[i];
                    
                    if (temp != array[i])
                    {
                        result = false;
                        break;
                    }
                }
            }

            Assert.IsTrue(result);
        }

        [Test]
        public void HexOffset()
        {
            const int offset = 3;
            int size = 1 + (offset << 1);
            var refStruct = hexC.HexesInOffset(offset);
            var array = hexC.GetHexesInOffset(offset);
            bool result = refStruct.Count == array.Length;

            if (result)
            {
                for (int w = 0; w < size; ++w)
                {
                    for (int h = 0; h < size; ++h)
                    {
                        var a = refStruct[w, h];
                        var b = array[w, h];

                        if (a != b)
                        {
                            Assert.IsTrue(false);
                        }
                    }
                }
            }

            Assert.IsTrue(result);
        }

        [Test]
        public void HexRotation()
        {
            for (int i = 0; i < 6; i++)
            {
                var rotateHex = hexA.RotateHex(hexB, i);
                var direction = hexA.GetDirectionToNeighbourHex(hexB);
                var opposite = direction.Rotate(i);
                var oppositeHex = hexA.GetHexAtDirection(opposite);
                Assert.IsTrue(rotateHex == oppositeHex);
            }
        }

        [Test]
        public void HexFromHash()
        {
            var hash = hexC.GetHashCode();
            var fromHash = Hex.FromHashCode(hash);
            Assert.IsTrue(hexC == fromHash);
        }
    }
}
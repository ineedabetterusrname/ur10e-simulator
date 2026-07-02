import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export function createScene(container) {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x11141a);
  scene.fog = new THREE.Fog(0x11141a, 8, 20);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 60);
  camera.position.set(2.4, 1.9, 2.6);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0.55, 0);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.maxPolarAngle = Math.PI * 0.52;
  controls.minDistance = 0.6;
  controls.maxDistance = 12;

  scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x3a3a42, 0.75));
  const key = new THREE.DirectionalLight(0xffffff, 2.2);
  key.position.set(4, 6, 3);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.camera.left = key.shadow.camera.bottom = -3;
  key.shadow.camera.right = key.shadow.camera.top = 3;
  key.shadow.bias = -0.0004;
  scene.add(key);
  const rim = new THREE.DirectionalLight(0x6fb7ff, 0.5);
  rim.position.set(-4, 3, -4);
  scene.add(rim);

  const ground = new THREE.Mesh(
    new THREE.CircleGeometry(6, 72),
    new THREE.MeshStandardMaterial({ color: 0x272b32, metalness: 0.1, roughness: 0.9 })
  );
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);
  const grid = new THREE.GridHelper(12, 48, 0x3d444e, 0x30353d);
  grid.position.y = 0.002;
  scene.add(grid);

  function resize() {
    const w = container.clientWidth || window.innerWidth;
    const h = container.clientHeight || window.innerHeight;
    renderer.setSize(w, h);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  window.addEventListener('resize', resize);
  resize();

  const pipFrame = document.getElementById('pip-frame');

  function render(pipCamera) {
    renderer.setScissorTest(false);
    renderer.render(scene, camera);
    const showPip = !!pipCamera;
    pipFrame.classList.toggle('hidden', !showPip);
    if (showPip) {
      const w = Math.round(renderer.domElement.clientWidth * 0.22);
      const pw = Math.max(220, Math.min(360, w));
      const ph = Math.round((pw * 9) / 16);
      const x = renderer.domElement.clientWidth - pw - 16;
      const y = 52;
      const dpr = renderer.getPixelRatio();
      renderer.setViewport(x, y, pw, ph);
      renderer.setScissor(x, y, pw, ph);
      renderer.setScissorTest(true);
      renderer.render(scene, pipCamera);
      renderer.setScissorTest(false);
      renderer.setViewport(0, 0, renderer.domElement.width / dpr, renderer.domElement.height / dpr);
      pipFrame.style.right = '16px';
      pipFrame.style.bottom = `${y}px`;
      pipFrame.style.width = `${pw}px`;
      pipFrame.style.height = `${ph}px`;
    }
  }

  return { renderer, scene, camera, controls, render };
}

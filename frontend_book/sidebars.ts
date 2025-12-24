import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Main sidebar for the textbook
  tutorialSidebar: [
    'intro',
    'setup-guide',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsed: false,
      items: [
        'module-1/chapter-1-ros2-fundamentals',
        'module-1/chapter-2-python-agents-controllers',
        'module-1/chapter-3-humanoid-modeling-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      collapsed: false,
      items: [
        'module-2/index',
        'module-2/chapter-1-gazebo-physics',
        'module-2/chapter-2-unity-rendering',
        'module-2/chapter-3-sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      collapsed: false,
      items: [
        'module-3/index',
        'module-3/chapter-1-isaac-sim',
        'module-3/chapter-2-isaac-ros',
        'module-3/chapter-3-nav2-bipedal',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      collapsed: false,
      items: [
        'module-4/index',
        'module-4/chapter-1-voice-to-action',
        'module-4/chapter-2-cognitive-planning',
        'module-4/chapter-3-capstone-vla',
      ],
    },
  ],
};

export default sidebars;

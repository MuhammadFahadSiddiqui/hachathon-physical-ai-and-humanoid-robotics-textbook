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
  ],
};

export default sidebars;

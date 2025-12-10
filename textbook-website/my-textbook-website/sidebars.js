// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Textbook sidebar with all chapters in proper order
  textbookSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics Textbook',
      collapsed: false,
      items: [
        'chapter-01-introduction',
        'chapter-02-biomechanics',
        'chapter-03-sensorimotor',
        'chapter-04-cognitive-architecture',
        'chapter-05-actuation',
        'chapter-06-control-theory',
        'chapter-07-locomotion',
        'chapter-08-manipulation',
        'chapter-09-perception',
        'chapter-10-learning',
        'chapter-11-hri',
        'chapter-12-simulation',
        'chapter-13-hardware',
        'chapter-14-energy',
        'chapter-15-safety',
        'chapter-16-applications',
        'chapter-17-future-directions',
        'chapter-18-ethics',
        'chapter-18-conclusion',
        'textbook-summary'
      ],
    },
  ],
};

export default sidebars;

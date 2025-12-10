// Chapter Navigation Loader
// This script dynamically loads the chapter navigation component

(function() {
  'use strict';

  // Wait for the page to load
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initializeChapterNavigation);
  } else {
    initializeChapterNavigation();
  }

  function initializeChapterNavigation() {
    // Check if we're on a docs page
    if (!window.location.pathname.includes('/docs/')) {
      return;
    }

    // Create a container for the chapter navigation
    const navContainer = document.createElement('div');
    navContainer.id = 'chapter-navigation-container';
    document.body.appendChild(navContainer);

    // Since we can't directly use React in a plain JS file,
    // we'll create a simpler version of the chapter navigation
    createSimpleChapterNavigation();
  }

  function createSimpleChapterNavigation() {
    // Chapter data
    const chapters = [
      { id: 'chapter-01-introduction', title: 'Introduction to Physical AI and Humanoid Robotics', path: '/docs/chapter-01-introduction' },
      { id: 'chapter-02-biomechanics', title: 'Biomechanics and Human Movement Principles', path: '/docs/chapter-02-biomechanics' },
      { id: 'chapter-03-sensorimotor', title: 'Sensorimotor Integration in Physical AI', path: '/docs/chapter-03-sensorimotor' },
      { id: 'chapter-04-cognitive-architecture', title: 'Cognitive Architecture for Humanoid Systems', path: '/docs/chapter-04-cognitive-architecture' },
      { id: 'chapter-05-actuation', title: 'Actuation Systems and Artificial Muscles', path: '/docs/chapter-05-actuation' },
      { id: 'chapter-06-control-theory', title: 'Control Theory for Humanoid Robots', path: '/docs/chapter-06-control-theory' },
      { id: 'chapter-07-locomotion', title: 'Locomotion and Bipedal Walking', path: '/docs/chapter-07-locomotion' },
      { id: 'chapter-08-manipulation', title: 'Manipulation and Dexterous Control', path: '/docs/chapter-08-manipulation' },
      { id: 'chapter-09-perception', title: 'Perception Systems and Environmental Understanding', path: '/docs/chapter-09-perception' },
      { id: 'chapter-10-learning', title: 'Learning and Adaptation in Physical Systems', path: '/docs/chapter-10-learning' },
      { id: 'chapter-11-hri', title: 'Human-Robot Interaction and Social Robotics', path: '/docs/chapter-11-hri' },
      { id: 'chapter-12-simulation', title: 'Simulation and Modeling for Physical AI', path: '/docs/chapter-12-simulation' },
      { id: 'chapter-13-hardware', title: 'Hardware Design and Integration', path: '/docs/chapter-13-hardware' },
      { id: 'chapter-14-energy', title: 'Energy Systems and Power Management', path: '/docs/chapter-14-energy' },
      { id: 'chapter-15-safety', title: 'Safety and Compliance in Physical AI', path: '/docs/chapter-15-safety' },
      { id: 'chapter-16-applications', title: 'Applications and Case Studies', path: '/docs/chapter-16-applications' },
      { id: 'chapter-17-future-directions', title: 'Future Directions and Research Challenges', path: '/docs/chapter-17-future-directions' },
      { id: 'chapter-18-ethics', title: 'Ethics and Governance of Humanoid Systems', path: '/docs/chapter-18-ethics' },
      { id: 'chapter-18-conclusion', title: 'Conclusion and Future Outlook', path: '/docs/chapter-18-conclusion' },
      { id: 'textbook-summary', title: 'Physical AI & Humanoid Robotics - Textbook Summary', path: '/docs/textbook-summary' }
    ];

    // Find current chapter index
    const currentPath = window.location.pathname;
    const currentChapterIndex = chapters.findIndex(chapter => currentPath.includes(chapter.id));

    // Get previous and next chapters
    const previousChapter = currentChapterIndex > 0 ? chapters[currentChapterIndex - 1] : null;
    const nextChapter = currentChapterIndex < chapters.length - 1 ? chapters[currentChapterIndex + 1] : null;

    // Create the chapter navigation HTML
    const navHTML = `
      <div id="chapter-nav-widget" style="
        position: fixed;
        bottom: 20px;
        right: 20px;
        z-index: 1000;
      ">
        <button id="chapter-nav-toggle" style="
          width: 60px;
          height: 60px;
          border-radius: 50%;
          border: none;
          background-color: #1a5d1a;
          color: white;
          font-size: 24px;
          cursor: pointer;
          box-shadow: 0 4px 8px rgba(0,0,0,0.2);
          display: flex;
          align-items: center;
          justify-content: center;
        ">📚</button>

        <div id="chapter-nav-panel" style="
          display: none;
          position: absolute;
          bottom: 70px;
          right: 0;
          width: 400px;
          max-height: 70vh;
          background-color: white;
          border-radius: 12px;
          box-shadow: 0 8px 24px rgba(0,0,0,0.15);
          overflow: hidden;
          flex-direction: column;
        ">
          <div class="chapter-nav-header" style="
            background-color: #1a5d1a;
            color: white;
            padding: 15px 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
          ">
            <h3 style="margin: 0; font-size: 16px; font-weight: 600;">Table of Contents</h3>
            <button id="chapter-nav-close" style="
              background: none;
              border: none;
              color: white;
              font-size: 24px;
              cursor: pointer;
              padding: 0;
              width: 30px;
              height: 30px;
              display: flex;
              align-items: center;
              justify-content: center;
            ">×</button>
          </div>

          <div class="chapter-nav-list" style="
            flex: 1;
            overflow-y: auto;
            padding: 10px 0;
          ">${chapters.map((chapter, index) => `
            <a href="${chapter.path}" class="chapter-nav-item ${currentPath.includes(chapter.id) ? 'active' : ''}" style="
              display: flex;
              align-items: center;
              padding: 12px 20px;
              text-decoration: none;
              color: #333;
              transition: background-color 0.2s ease;
              ${currentPath.includes(chapter.id) ? 'background-color: rgba(26, 93, 26, 0.2); font-weight: 600;' : ''}
            ">
              <span class="chapter-number" style="
                margin-right: 10px;
                color: #1a5d1a;
                font-weight: 600;
                min-width: 30px;
              ">${index + 1}.</span>
              <span class="chapter-title" style="
                flex: 1;
                white-space: nowrap;
                overflow: hidden;
                text-overflow: ellipsis;
              ">${chapter.title}</span>
              ${currentPath.includes(chapter.id) ? '<span class="current-indicator" style="margin-left: 10px; color: #1a5d1a;">●</span>' : ''}
            </a>
          `).join('')}</div>

          ${(previousChapter || nextChapter) ? `
          <div class="chapter-nav-pagination" style="
            border-top: 1px solid #eee;
            padding: 15px;
            display: flex;
            flex-direction: column;
            gap: 10px;
          ">
            ${previousChapter ? `
            <a href="${previousChapter.path}" class="chapter-nav-prev" style="
              padding: 10px 15px;
              border-radius: 6px;
              text-decoration: none;
              font-size: 14px;
              background-color: #f8f9fa;
              color: #333;
            ">← Previous: ${previousChapter.title}</a>
            ` : ''}
            ${nextChapter ? `
            <a href="${nextChapter.path}" class="chapter-nav-next" style="
              padding: 10px 15px;
              border-radius: 6px;
              text-decoration: none;
              font-size: 14px;
              background-color: #1a5d1a;
              color: white;
            ">Next: ${nextChapter.title} →</a>
            ` : ''}
          </div>
          ` : ''}
        </div>
      </div>
    `;

    // Add the HTML to the page
    document.body.insertAdjacentHTML('beforeend', navHTML);

    // Add event listeners
    const toggleBtn = document.getElementById('chapter-nav-toggle');
    const navPanel = document.getElementById('chapter-nav-panel');
    const closeBtn = document.getElementById('chapter-nav-close');

    toggleBtn.addEventListener('click', function() {
      navPanel.style.display = navPanel.style.display === 'flex' ? 'none' : 'flex';
    });

    closeBtn.addEventListener('click', function() {
      navPanel.style.display = 'none';
    });

    // Close when clicking outside
    document.addEventListener('click', function(event) {
      if (!navPanel.contains(event.target) && !toggleBtn.contains(event.target)) {
        navPanel.style.display = 'none';
      }
    });
  }
})();
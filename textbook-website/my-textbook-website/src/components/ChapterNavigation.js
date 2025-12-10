import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import { useColorMode } from '@docusaurus/theme-common';

const ChapterNavigation = () => {
  const location = useLocation();
  const { colorMode } = useColorMode();
  const [isVisible, setIsVisible] = useState(false);

  // Define the chapter structure
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
  const currentChapterIndex = chapters.findIndex(chapter =>
    location.pathname.includes(chapter.id)
  );

  // Get previous and next chapters
  const previousChapter = currentChapterIndex > 0 ? chapters[currentChapterIndex - 1] : null;
  const nextChapter = currentChapterIndex < chapters.length - 1 ? chapters[currentChapterIndex + 1] : null;

  // Toggle visibility of navigation
  const toggleNavigation = () => {
    setIsVisible(!isVisible);
  };

  // Auto-hide navigation after 5 seconds of inactivity
  useEffect(() => {
    let timeoutId;
    const resetTimeout = () => {
      setIsVisible(false);
      clearTimeout(timeoutId);
      timeoutId = setTimeout(() => setIsVisible(false), 5000);
    };

    resetTimeout();

    window.addEventListener('scroll', resetTimeout);
    window.addEventListener('mousemove', resetTimeout);

    return () => {
      window.removeEventListener('scroll', resetTimeout);
      window.removeEventListener('mousemove', resetTimeout);
      clearTimeout(timeoutId);
    };
  }, []);

  return (
    <div className={`chapter-navigation ${colorMode}`}>
      {/* Floating chapter navigation button */}
      <button
        className={`chapter-nav-toggle ${colorMode}`}
        onClick={toggleNavigation}
        title="Chapter Navigation"
      >
        📚
      </button>

      {/* Dynamic chapter navigation panel */}
      {(isVisible || location.pathname.includes('/docs/')) && (
        <div className={`chapter-nav-panel ${colorMode}`}>
          <div className="chapter-nav-header">
            <h3>Table of Contents</h3>
            <button
              className="chapter-nav-close"
              onClick={() => setIsVisible(false)}
            >
              ×
            </button>
          </div>

          <div className="chapter-nav-list">
            {chapters.map((chapter, index) => (
              <a
                key={chapter.id}
                href={chapter.path}
                className={`chapter-nav-item ${
                  location.pathname.includes(chapter.id) ? 'active' : ''
                }`}
              >
                <span className="chapter-number">{index + 1}.</span>
                <span className="chapter-title">{chapter.title}</span>
                {location.pathname.includes(chapter.id) && (
                  <span className="current-indicator">●</span>
                )}
              </a>
            ))}
          </div>

          {/* Previous/Next navigation */}
          {(previousChapter || nextChapter) && (
            <div className="chapter-nav-pagination">
              {previousChapter && (
                <a
                  href={previousChapter.path}
                  className="chapter-nav-prev"
                >
                  ← Previous: {previousChapter.title}
                </a>
              )}
              {nextChapter && (
                <a
                  href={nextChapter.path}
                  className="chapter-nav-next"
                >
                  Next: {nextChapter.title} →
                </a>
              )}
            </div>
          )}
        </div>
      )}

      <style jsx>{`
        .chapter-navigation {
          position: fixed;
          bottom: 20px;
          right: 20px;
          z-index: 1000;
        }

        .chapter-nav-toggle {
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
          transition: all 0.3s ease;
        }

        .chapter-nav-toggle:hover {
          transform: scale(1.1);
          box-shadow: 0 6px 12px rgba(0,0,0,0.3);
        }

        .chapter-nav-toggle.dark {
          background-color: #4caf50;
        }

        .chapter-nav-panel {
          position: absolute;
          bottom: 70px;
          right: 0;
          width: 400px;
          max-height: 70vh;
          background-color: white;
          border-radius: 12px;
          box-shadow: 0 8px 24px rgba(0,0,0,0.15);
          overflow: hidden;
          display: flex;
          flex-direction: column;
        }

        .chapter-nav-panel.dark {
          background-color: #1e1e1e;
          color: white;
        }

        .chapter-nav-header {
          background-color: #1a5d1a;
          color: white;
          padding: 15px 20px;
          display: flex;
          justify-content: space-between;
          align-items: center;
        }

        .chapter-nav-header.dark {
          background-color: #4caf50;
        }

        .chapter-nav-header h3 {
          margin: 0;
          font-size: 16px;
          font-weight: 600;
        }

        .chapter-nav-close {
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
        }

        .chapter-nav-list {
          flex: 1;
          overflow-y: auto;
          padding: 10px 0;
        }

        .chapter-nav-item {
          display: flex;
          align-items: center;
          padding: 12px 20px;
          text-decoration: none;
          color: #333;
          transition: background-color 0.2s ease;
        }

        .chapter-nav-item:hover {
          background-color: rgba(26, 93, 26, 0.1);
        }

        .chapter-nav-item.active {
          background-color: rgba(26, 93, 26, 0.2);
          font-weight: 600;
        }

        .chapter-nav-item.dark {
          color: #e0e0e0;
        }

        .chapter-nav-item.dark:hover {
          background-color: rgba(76, 175, 80, 0.1);
        }

        .chapter-nav-item.dark.active {
          background-color: rgba(76, 175, 80, 0.2);
        }

        .chapter-number {
          margin-right: 10px;
          color: #1a5d1a;
          font-weight: 600;
          min-width: 30px;
        }

        .chapter-nav-item.dark .chapter-number {
          color: #4caf50;
        }

        .chapter-title {
          flex: 1;
          white-space: nowrap;
          overflow: hidden;
          text-overflow: ellipsis;
        }

        .current-indicator {
          margin-left: 10px;
          color: #1a5d1a;
          font-size: 12px;
        }

        .chapter-nav-item.dark .current-indicator {
          color: #4caf50;
        }

        .chapter-nav-pagination {
          border-top: 1px solid #eee;
          padding: 15px;
          display: flex;
          flex-direction: column;
          gap: 10px;
        }

        .chapter-nav-pagination.dark {
          border-top: 1px solid #444;
        }

        .chapter-nav-prev,
        .chapter-nav-next {
          padding: 10px 15px;
          border-radius: 6px;
          text-decoration: none;
          font-size: 14px;
          transition: background-color 0.2s ease;
        }

        .chapter-nav-prev {
          background-color: #f8f9fa;
          color: #333;
        }

        .chapter-nav-next {
          background-color: #1a5d1a;
          color: white;
        }

        .chapter-nav-prev:hover,
        .chapter-nav-next:hover {
          text-decoration: none;
        }

        .chapter-nav-prev:hover {
          background-color: #e9ecef;
        }

        .chapter-nav-next:hover {
          background-color: #0f3d0f;
        }

        .chapter-nav-prev.dark {
          background-color: #2d2d2d;
          color: #e0e0e0;
        }

        .chapter-nav-next.dark {
          background-color: #4caf50;
          color: #121212;
        }

        .chapter-nav-prev.dark:hover {
          background-color: #3d3d3d;
        }

        .chapter-nav-next.dark:hover {
          background-color: #66bb6a;
        }

        /* Responsive design */
        @media (max-width: 768px) {
          .chapter-nav-panel {
            width: 320px;
            right: -20px;
          }

          .chapter-nav-item {
            padding: 10px 15px;
            font-size: 14px;
          }
        }
      `}</style>
    </div>
  );
};

export default ChapterNavigation;
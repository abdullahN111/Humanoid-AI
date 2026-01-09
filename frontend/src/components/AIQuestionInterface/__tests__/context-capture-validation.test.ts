import { ContextCapture } from '../ContextCapture/contextCapture';

describe('Context Capture Validation Across Different Pages', () => {
  // Mock window and document for different page scenarios
  const originalWindow = { ...global.window };
  const originalDocument = { ...global.document };
  const originalLocation = { ...window.location };

  beforeEach(() => {
    // Reset mocks for each test
    Object.defineProperty(window, 'location', {
      writable: true,
      value: { ...originalLocation }
    });
  });

  afterEach(() => {
    // Restore original window and document after each test
    Object.defineProperty(window, 'location', {
      writable: true,
      value: originalLocation
    });
  });

  test('captures context correctly on a documentation page', () => {
    // Mock a documentation page
    Object.defineProperty(window, 'location', {
      value: {
        href: 'https://example.com/docs/module-1/intro',
        pathname: '/docs/module-1/intro'
      },
      writable: true
    });

    Object.defineProperty(document, 'title', {
      value: 'Module 1 Introduction - ROS 2 Documentation',
      writable: true
    });

    // Mock selection
    const mockSelection = { toString: () => 'selected documentation text' } as Selection;
    Object.defineProperty(window, 'getSelection', {
      value: () => mockSelection,
      writable: true
    });

    const context = ContextCapture.captureContext();

    expect(context.currentPageUrl).toBe('https://example.com/docs/module-1/intro');
    expect(context.currentPageTitle).toBe('Module 1 Introduction - ROS 2 Documentation');
    expect(context.currentPageSection).toBe('intro'); // Should extract from URL
    expect(context.selectedText).toBe('selected documentation text');
  });

  test('captures context correctly on a different documentation page', () => {
    // Mock a different documentation page
    Object.defineProperty(window, 'location', {
      value: {
        href: 'https://example.com/docs/module-2/chapter-3',
        pathname: '/docs/module-2/chapter-3'
      },
      writable: true
    });

    Object.defineProperty(document, 'title', {
      value: 'Chapter 3: Advanced Concepts - ROS 2 Guide',
      writable: true
    });

    // Mock no selection
    const mockSelection = { toString: () => '' } as Selection;
    Object.defineProperty(window, 'getSelection', {
      value: () => mockSelection,
      writable: true
    });

    const context = ContextCapture.captureContext();

    expect(context.currentPageUrl).toBe('https://example.com/docs/module-2/chapter-3');
    expect(context.currentPageTitle).toBe('Chapter 3: Advanced Concepts - ROS 2 Guide');
    expect(context.currentPageSection).toBe('chapter-3'); // Should extract from URL
    expect(context.selectedText).toBe('');
  });

  test('captures context correctly on a deep nested page', () => {
    // Mock a deeply nested documentation page
    Object.defineProperty(window, 'location', {
      value: {
        href: 'https://example.com/docs/module-3/advanced-topics/simulation/gazebo-integration',
        pathname: '/docs/module-3/advanced-topics/simulation/gazebo-integration'
      },
      writable: true
    });

    Object.defineProperty(document, 'title', {
      value: 'Gazebo Integration - Advanced Simulation Topics',
      writable: true
    });

    // Mock selection with longer text
    const mockSelection = { toString: () => 'This is a longer selected text from a deeply nested documentation page' } as Selection;
    Object.defineProperty(window, 'getSelection', {
      value: () => mockSelection,
      writable: true
    });

    const context = ContextCapture.captureContext();

    expect(context.currentPageUrl).toBe('https://example.com/docs/module-3/advanced-topics/simulation/gazebo-integration');
    expect(context.currentPageTitle).toBe('Gazebo Integration - Advanced Simulation Topics');
    expect(context.currentPageSection).toBe('gazebo-integration'); // Should extract from URL
    expect(context.selectedText).toBe('This is a longer selected text from a deeply nested documentation page');
  });

  test('handles pages with query parameters', () => {
    // Mock a page with query parameters
    Object.defineProperty(window, 'location', {
      value: {
        href: 'https://example.com/docs/tutorials?category=beginner&page=5',
        pathname: '/docs/tutorials',
        search: '?category=beginner&page=5'
      },
      writable: true
    });

    Object.defineProperty(document, 'title', {
      value: 'Tutorials - ROS 2 Learning Path',
      writable: true
    });

    const mockSelection = { toString: () => 'selected text from tutorial' } as Selection;
    Object.defineProperty(window, 'getSelection', {
      value: () => mockSelection,
      writable: true
    });

    const context = ContextCapture.captureContext();

    expect(context.currentPageUrl).toBe('https://example.com/docs/tutorials?category=beginner&page=5');
    expect(context.currentPageTitle).toBe('Tutorials - ROS 2 Learning Path');
    expect(context.currentPageSection).toBe('tutorials'); // Should extract from pathname
    expect(context.selectedText).toBe('selected text from tutorial');
  });

  test('handles pages with hash fragments', () => {
    // Mock a page with hash fragment
    Object.defineProperty(window, 'location', {
      value: {
        href: 'https://example.com/docs/api-reference#robot-state',
        pathname: '/docs/api-reference',
        hash: '#robot-state'
      },
      writable: true
    });

    Object.defineProperty(document, 'title', {
      value: 'API Reference - ROS 2 Documentation',
      writable: true
    });

    const mockSelection = { toString: () => 'API reference text' } as Selection;
    Object.defineProperty(window, 'getSelection', {
      value: () => mockSelection,
      writable: true
    });

    const context = ContextCapture.captureContext();

    expect(context.currentPageUrl).toBe('https://example.com/docs/api-reference#robot-state');
    expect(context.currentPageTitle).toBe('API Reference - ROS 2 Documentation');
    expect(context.currentPageSection).toBe('api-reference'); // Should extract from pathname
    expect(context.selectedText).toBe('API reference text');
  });

  test('extracts description from meta tags when available', () => {
    // Create a mock meta element for description
    const metaDescription = document.createElement('meta');
    metaDescription.name = 'description';
    metaDescription.content = 'This is a sample documentation page about ROS 2 fundamentals';

    jest.spyOn(document, 'querySelector').mockImplementation((selector) => {
      if (selector === 'meta[name="description"]') {
        return metaDescription;
      }
      return null;
    });

    Object.defineProperty(window, 'location', {
      value: {
        href: 'https://example.com/docs/fundamentals',
        pathname: '/docs/fundamentals'
      },
      writable: true
    });

    Object.defineProperty(document, 'title', {
      value: 'ROS 2 Fundamentals',
      writable: true
    });

    const mockSelection = { toString: () => '' } as Selection;
    Object.defineProperty(window, 'getSelection', {
      value: () => mockSelection,
      writable: true
    });

    const context = ContextCapture.captureContext();

    expect(context.currentPageUrl).toBe('https://example.com/docs/fundamentals');
    expect(context.currentPageTitle).toBe('ROS 2 Fundamentals');
    expect(context.currentPageDescription).toBe('This is a sample documentation page about ROS 2 fundamentals');
    expect(context.currentPageSection).toBe('fundamentals');
    expect(context.selectedText).toBe('');
  });

  test('works correctly when running in a Node.js environment (no window/document)', () => {
    // Temporarily remove window and document to simulate Node.js environment
    const windowBackup = global.window;
    const documentBackup = global.document;

    // @ts-ignore
    delete global.window;
    // @ts-ignore
    delete global.document;

    try {
      // These should return empty strings when window/document are not available
      expect(ContextCapture.getCurrentPageUrl()).toBe('');
      expect(ContextCapture.getCurrentPageTitle()).toBe('');
      expect(ContextCapture.getSelectedText()).toBe('');
    } finally {
      // Restore window and document
      global.window = windowBackup;
      global.document = documentBackup;
    }
  });
});